use std::time::Duration;

use ahash::HashMap;
use bevy::prelude::*;
use common::{
    bundles::{MotorBundle, PwmActuatorBundle, RobotActuatorBundle},
    components::{
        ActualForce, ActualMovement, Armed, CurrentDraw, JerkLimit, MotorContribution,
        MotorDefinition, Motors, MovementAxisMaximums, MovementContribution, MovementCurrentCap,
        PwmChannel, PwmManualControl, PwmSignal, RobotId, TargetForce, TargetMovement,
    },
    ecs_sync::{NetId, Replicate},
    types::units::{Amperes, Newtons},
};
use motor_math::{
    blue_rov::BlueRovMotorId,
    blue_rov_heavy::HeavyMotorId,
    glam::MovementGlam,
    motor_preformance::{self, Interpolation, MotorData, MotorRecord},
    solve::{self, reverse},
    x3d::X3dMotorId,
    Direction, ErasedMotorId,
};
use stable_hashmap::StableHashMap;

use crate::{
    config::{MotorConfigDefinition, RobotConfig},
    plugins::core::robot::{LocalRobot, LocalRobotMarker},
};

pub struct ThrusterPlugin;

impl Plugin for ThrusterPlugin {
    fn build(&self, app: &mut App) {
        // FIXME(low): This is kinda bad
        let motor_data =
            motor_preformance::read_motor_data("motor_data.csv").expect("Read motor data");

        // TODO(mid): Update motor config when motor definitions change
        app.add_systems(Startup, (create_motors, setup_motor_math))
            .add_systems(
                Update,
                (
                    update_axis_maximums,
                    accumulate_movements,
                    accumulate_motor_forces.after(accumulate_movements),
                ),
            )
            .insert_resource(MotorDataRes(motor_data));
    }
}

#[derive(Resource)]
pub struct MotorDataRes(pub MotorData);

fn create_motors(mut cmds: Commands, robot: Res<LocalRobot>, config: Res<RobotConfig>) {
    let (motors, motor_config) = config.motor_config.flatten(config.center_of_mass);

    info!("Generating motor config");

    cmds.entity(robot.entity).insert(RobotActuatorBundle {
        movement_target: TargetMovement(Default::default()),
        movement_actual: ActualMovement(Default::default()),
        motor_config: Motors(motor_config),
        axis_maximums: MovementAxisMaximums(Default::default()),
        current_cap: MovementCurrentCap(config.motor_amperage_budget.into()),
        armed: Armed::Disarmed,
    });

    for (motor_id, motor, pwm_channel) in motors {
        let name = match config.motor_config {
            MotorConfigDefinition::X3d(_) => {
                format!(
                    "{:?} ({motor_id})",
                    X3dMotorId::try_from(motor_id).expect("Bad motor id for config")
                )
            }
            MotorConfigDefinition::BlueRov(_) => {
                format!(
                    "{:?} ({motor_id})",
                    BlueRovMotorId::try_from(motor_id).expect("Bad motor id for config")
                )
            }
            MotorConfigDefinition::Heavy(_) => {
                format!(
                    "{:?} ({motor_id})",
                    HeavyMotorId::try_from(motor_id).expect("Bad motor id for config")
                )
            }
            MotorConfigDefinition::Custom(_) => format!("Motor {motor_id}"),
        };

        cmds.spawn((
            MotorBundle {
                actuator: PwmActuatorBundle {
                    name: Name::new(name),
                    pwm_channel: PwmChannel(pwm_channel),
                    pwm_signal: PwmSignal(Duration::from_micros(1500)),
                    robot: RobotId(robot.net_id),
                },
                motor: MotorDefinition(motor_id, motor),
                target_force: TargetForce(0.0f32.into()),
                actual_force: ActualForce(0.0f32.into()),
                current_draw: CurrentDraw(0.0f32.into()),
            },
            Replicate,
        ));
    }
}

fn setup_motor_math(mut cmds: Commands, config: Res<RobotConfig>, robot: Res<LocalRobot>) {
    cmds.entity(robot.entity)
        .insert(JerkLimit(config.jerk_limit));
}

fn update_axis_maximums(
    mut cmds: Commands,
    robot: Query<
        (Entity, &MovementCurrentCap, &Motors),
        (With<LocalRobotMarker>, Changed<MovementCurrentCap>),
    >,
    motor_data: Res<MotorDataRes>,
) {
    for (entity, current_cap, motor_config) in &robot {
        let motor_config = &motor_config.0;
        let motor_data = &motor_data.0;
        let current_cap = current_cap.0 .0;

        let maximums = reverse::axis_maximums(motor_config, motor_data, current_cap as _, 0.05)
            .into_iter()
            .map(|(key, value)| (key, Newtons(value as _)))
            .collect();

        info!("Updated motor axis maximums to {maximums:?} at {current_cap:.2}A");

        cmds.entity(entity).insert(MovementAxisMaximums(maximums));
    }
}

fn accumulate_movements(
    mut cmds: Commands,
    robot: Query<(Entity, &NetId, &Motors), (With<LocalRobotMarker>, Without<PwmManualControl>)>,
    movements: Query<(&RobotId, &MovementContribution)>,

    motor_data: Res<MotorDataRes>,
) {
    let Ok((entity, net_id, Motors(motor_config))) = robot.get_single() else {
        return;
    };
    let mut robot = cmds.entity(entity);

    let mut total_movement = MovementGlam::default();

    for (RobotId(robot_net_id), movement) in &movements {
        if robot_net_id == net_id {
            total_movement += movement.0;
        }
    }

    let forces = solve::reverse::reverse_solve(total_movement.into(), motor_config);
    let motor_cmds = solve::reverse::forces_to_cmds(forces, motor_config, &motor_data.0);
    let forces = motor_cmds
        .into_iter()
        .map(|(motor, cmd)| (motor, Newtons(cmd.force as _)))
        .collect();

    robot.insert(MotorContribution(forces));
}

// TODO(mid): Split into smaller systems
fn accumulate_motor_forces(
    mut cmds: Commands,
    mut last_movement: Local<StableHashMap<ErasedMotorId, MotorRecord<motor_math::FloatType>>>,

    robot: Query<
        (Entity, &NetId, &Motors, &MovementCurrentCap, &JerkLimit),
        (With<LocalRobotMarker>, Without<PwmManualControl>),
    >,
    motor_forces: Query<(&RobotId, &MotorContribution)>,
    motors: Query<(Entity, &MotorDefinition, &RobotId)>,

    time: Res<Time<Real>>,
    motor_data: Res<MotorDataRes>,
) {
    let Ok((
        entity,
        &net_id,
        Motors(motor_config),
        &MovementCurrentCap(current_cap),
        &JerkLimit(jerk_limit),
    )) = robot.get_single()
    else {
        return;
    };
    let mut robot = cmds.entity(entity);

    let mut all_forces = StableHashMap::default();

    for (&RobotId(robot_net_id), motor_force_contributions) in &motor_forces {
        if robot_net_id == net_id {
            for (motor, force) in &motor_force_contributions.0 {
                *all_forces.entry(*motor).or_default() += force.0 as motor_math::FloatType;
            }
        }
    }

    let target_movement = solve::forward::forward_solve(motor_config, &all_forces);
    robot.insert(TargetMovement(target_movement.into()));

    let motor_cmds = all_forces
        .iter()
        .map(|(motor, force)| {
            let direction = motor_config
                .motor(motor)
                .map(|it| it.direction)
                .unwrap_or(Direction::Clockwise);

            (
                *motor,
                motor_data.0.lookup_by_force(
                    *force,
                    Interpolation::LerpDirection(direction),
                    false,
                ),
            )
        })
        .collect();

    let motor_cmds = solve::reverse::clamp_amperage(
        motor_cmds,
        motor_config,
        &motor_data.0,
        current_cap.0 as _,
        0.01,
    );

    // Implement slew rate limiting
    let motor_cmds = {
        let slew_motor_cmds = motor_cmds
            .iter()
            .map(|(motor, record)| {
                if let Some(last) = last_movement.get(motor) {
                    let jerk_limit = jerk_limit * time.delta_seconds();
                    let delta = record.force - last.force;

                    if delta.abs() > jerk_limit as _ {
                        let direction = motor_config
                            .motor(motor)
                            .map(|it| it.direction)
                            .unwrap_or(Direction::Clockwise);

                        let clamped = delta.clamp(-jerk_limit as _, jerk_limit as _);
                        let new_record = motor_data.0.lookup_by_force(
                            clamped + last.force,
                            Interpolation::LerpDirection(direction),
                            false,
                        );

                        return (*motor, new_record);
                    }
                };

                (*motor, *record)
            })
            .collect();

        // FIXME: Why do we clamp amperage twice???
        solve::reverse::clamp_amperage(
            slew_motor_cmds,
            motor_config,
            &motor_data.0,
            current_cap.0 as _,
            0.01,
        )
    };

    let motor_forces = motor_cmds
        .iter()
        .map(|(motor, data)| (*motor, data.force))
        .collect();

    let actual_movement = solve::forward::forward_solve(motor_config, &motor_forces);
    robot.insert(ActualMovement(actual_movement.into()));

    for (motor_entity, MotorDefinition(id, _motor), &RobotId(robot_net_id)) in &motors {
        if robot_net_id == net_id {
            let mut motor = cmds.entity(motor_entity);

            // FIXME(mid): panics
            let target_force = all_forces.get(id);
            let actual_data = motor_cmds.get(id);

            // TODO(mid): Special case for 0

            if let (Some(target_force), Some(actual_data)) = (target_force, actual_data) {
                motor.insert((
                    TargetForce(Newtons(*target_force as _)),
                    ActualForce(Newtons(actual_data.force as _)),
                    CurrentDraw(Amperes(actual_data.current as _)),
                    PwmSignal(Duration::from_micros(actual_data.pwm as u64)),
                ));
            } else {
                motor.insert((
                    TargetForce(0.0.into()),
                    ActualForce(0.0.into()),
                    CurrentDraw(0.0.into()),
                    PwmSignal(Duration::from_micros(1500)),
                ));
            }
        }
    }

    *last_movement = motor_cmds;
}
