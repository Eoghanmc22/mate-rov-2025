use ahash::{HashMap, HashSet};
use bevy::prelude::*;
use common::{
    bundles::{ActuatorBundle, MotorBundle},
    components::{
        DisableMovementApi, GenericMotorId, MotorCameraReference, MotorContribution,
        MotorContributionMode, MotorSignal, MotorSignalType, MotorTargets, Motors, RobotId,
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
        },
    ) in servos
    {
        let mut entity = cmds.spawn((
            MotorBundle {
                actuator: ActuatorBundle {
                    name: Name::new(name.clone()),
                    channel: channel.into(),
                    signal: MotorSignal::Percent(0.0),
                    robot: RobotId(robot.net_id),
                    signal_type: MotorSignalType::Position,
                    // TODO : We need a way to get the actual range
                    signal_range: channel.default_signal_range(),
                },
                // TODO: This should prob be configurable
                mode: MotorContributionMode::ZerothOrder,
            },
            Replicate,
        ));
        if let Some(camera) = camera {
            entity.insert(MotorCameraReference {
                // TODO: this defeats the point of COW
                camera: camera.to_owned().into(),
            });
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
        &MotorSignalType,
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
            *all_inputs.entry(motor.clone()).or_default() += *input;
        }
    }

    let servos_by_id = servos
        .iter()
        .map(|it| (it.4, it))
        .collect::<HashMap<_, _>>();

    let mut full_reset = false;

    if !reset.is_empty() {
        full_reset = true;
        reset.clear();
    }

    let mut new_positions = last_positions.0.clone();
    let mut should_reset = HashSet::default();

    for event in reset_single.read() {
        new_positions.insert(event.0.clone(), 0.0);
        should_reset.insert(event.0.clone());
    }

    new_positions.extend(all_inputs.into_iter().flat_map(|(id, input)| {
        let (_, _, _, mode, _, _) = servos_by_id.get(&id)?;

        // TODO: Check if this is even right
        match mode {
            MotorContributionMode::ZerothOrder => Some((id, input)),
            MotorContributionMode::FirstOrder => {
                let last_position = if !full_reset && !should_reset.contains(&id) {
                    last_positions.0.get(&id).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
                Some((
                    id,
                    (last_position + input * time.delta_secs()).clamp(-1.0, 1.0),
                ))
            }
        }
    }));

    for (id, position) in &new_positions {
        let Some((servo, ..)) = servos_by_id.get(&*id) else {
            continue;
        };

        // let micros = 1500.0 + 400.0 * position.clamp(-1.0, 1.0);

        cmds.entity(*servo).insert(MotorSignal::Percent(*position));
    }

    cmds.entity(robot).insert(MotorTargets(new_positions));
}
