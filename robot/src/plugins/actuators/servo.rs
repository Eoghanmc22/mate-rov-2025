use ahash::{HashMap, HashSet};
use bevy::prelude::*;
use common::{
    bundles::{ActuatorBundle, MotorBundle},
    components::{
        DisableMovementApi, GenericMotorId, MotorCameraReference, MotorContribution,
        MotorContributionMode, MotorRawSignalRange, MotorSignal, MotorSignalType, MotorSlewRate,
        MotorTargets, Motors, RobotId,
    },
    ecs_sync::{NetId, Replicate},
    events::{ResetServo, ResetServos},
};
use motor_math::motor_preformance::MotorData;

use crate::{
    config::{RobotConfig, Servo},
    plugins::core::robot::{LocalRobot, LocalRobotMarker},
};

pub struct ServoPlugin;

impl Plugin for ServoPlugin {
    fn build(&self, app: &mut App) {
        // TODO(mid): Update motor config when motor definitions change
        app.add_systems(Startup, create_servos)
            .add_systems(Update, handle_servo_input);
    }
}

#[derive(Resource)]
pub struct MotorDataRes(pub MotorData);

fn create_servos(mut cmds: Commands, robot: Res<LocalRobot>, config: Res<RobotConfig>) {
    let servos = &config.servo_config.servos;

    // TODO: Make this a bundle
    cmds.entity(robot.entity).insert((
        Motors {
            ids: servos
                .iter()
                .map(|(_, servo)| servo.channel.into())
                .collect(),
        },
        MotorTargets::default(),
    ));

    for (
        name,
        &Servo {
            channel,
            ref camera,
            ref constraints,
            signal_type,
            control_mode,
            slew_rate,
        },
    ) in servos
    {
        let default_signal_range = channel.default_signal_range();
        let signal_range = if let Some(constraints) = constraints {
            let min_raw = default_signal_range.raw_from_percent(constraints.min);
            let max_raw = default_signal_range.raw_from_percent(constraints.max);

            MotorRawSignalRange {
                min: default_signal_range.min.max(min_raw),
                center: default_signal_range.center,
                max: default_signal_range.max.min(max_raw),
            }
        } else {
            default_signal_range
        };
        let mode = control_mode.unwrap_or_default();

        let mut entity = cmds.spawn((
            MotorBundle {
                actuator: ActuatorBundle {
                    name: Name::new(name.clone()),
                    channel: channel.into(),
                    signal: MotorSignal::Percent(0.0),
                    robot: RobotId(robot.net_id),
                    signal_type,
                    // TODO : We need a way to get the actual range
                    signal_range,
                },
                // TODO: This should prob be configurable
                mode,
            },
            Replicate,
        ));
        if let Some(camera) = camera {
            entity.insert(MotorCameraReference {
                // TODO: this defeats the point of COW
                camera: camera.to_owned().into(),
            });
        }
        if let Some(slew_rate) = slew_rate {
            entity.insert(slew_rate);
        }
    }
}

fn handle_servo_input(
    mut cmds: Commands,

    robot: Query<
        (Entity, &NetId, &MotorTargets),
        // FIXME: Should this really be `Without<DisableMovementApi>`
        (With<LocalRobotMarker>, Without<DisableMovementApi>),
    >,
    servo_inputs: Query<(&RobotId, &MotorContribution)>,
    // TODO
    servos: Query<(
        Entity,
        &Name,
        Option<&MotorSignal>,
        &MotorSignalType,
        Option<&MotorSlewRate>,
        &MotorContributionMode,
        &GenericMotorId,
        &RobotId,
    )>,

    mut reset: EventReader<ResetServos>,
    mut reset_single: EventReader<ResetServo>,

    time: Res<Time<Real>>,
) {
    let Ok((robot, &net_id, last_positions)) = robot.get_single() else {
        return;
    };

    let mut all_inputs = HashMap::<_, f32>::default();

    for (&RobotId(robot_net_id), servo_contribution) in &servo_inputs {
        if robot_net_id != net_id {
            continue;
        }

        for (motor, input) in &servo_contribution.0 {
            *all_inputs.entry(motor).or_default() += *input;
        }
    }

    let servos_by_id = servos
        .iter()
        .map(|it| (*it.6, it))
        .collect::<HashMap<GenericMotorId, _>>();

    let mut full_reset = false;

    if !reset.is_empty() {
        full_reset = true;
        reset.clear();
    }

    let mut new_positions = last_positions.0.clone();
    let mut should_reset = HashSet::default();

    for event in reset_single.read() {
        new_positions.insert(event.0, 0.0);
        should_reset.insert(event.0);
    }

    new_positions.extend(all_inputs.into_iter().flat_map(|(id, input)| {
        // This is terrifying
        let (_, _, _, _, _, mode, _, _) = servos_by_id.get(id)?;

        // TODO: Check if this is even right
        match mode {
            MotorContributionMode::ZerothOrder => Some((*id, input)),
            MotorContributionMode::FirstOrder => {
                let last_position = if !full_reset && !should_reset.contains(id) {
                    last_positions.0.get(id).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
                Some((
                    *id,
                    (last_position + input * time.delta_secs()).clamp(-1.0, 1.0),
                ))
            }
        }
    }));

    for (id, &position) in &new_positions {
        let Some((servo, _, last_signal, _, slew_rate, ..)) = servos_by_id.get(id) else {
            continue;
        };

        // TODO: make this implementation more flexable (ie support raw signals)
        let position = if let (
            Some(&MotorSignal::Percent(last_position)),
            Some(&MotorSlewRate(MotorSignal::Percent(slew_rate))),
        ) = (last_signal, slew_rate)
        {
            let slew_rate = slew_rate * time.delta_secs();
            let delta = position - last_position;

            if delta.abs() > slew_rate {
                last_position + delta.clamp(-slew_rate, slew_rate)
            } else {
                position
            }
        } else {
            position
        };

        // let micros = 1500.0 + 400.0 * position.clamp(-1.0, 1.0);

        cmds.entity(*servo).insert(MotorSignal::Percent(position));
    }

    cmds.entity(robot).insert(MotorTargets(new_positions));
}
