use std::f32::consts::{PI, TAU};

use bevy::prelude::*;
use common::{
    bundles::MovementContributionBundle,
    components::{
        Armed, DepthMeasurement, DepthTarget, MovementContribution, Orientation, OrientationTarget,
        PidConfig, PidResult, RobotId,
    },
    ecs_sync::Replicate,
    types::utils::PidController,
};
use glam::{vec3a, Vec3A};
use motor_math::glam::MovementGlam;
use serde::{Deserialize, Serialize};

use crate::{
    config::RobotConfig,
    plugins::{
        core::robot::{LocalRobot, LocalRobotMarker},
        sensors::{depth, orientation},
    },
};

pub struct StabilizePlugin;

impl Plugin for StabilizePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_stabalize);
        app.add_systems(Update, stabalize_system);
    }
}

#[derive(Component, Default)]
struct PidState(PidController);

#[derive(Component, Debug, Hash, PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum PidAxis {
    Depth,
    Yaw,
    Pitch,
    Roll,
}

impl PidAxis {
    fn get_unit_local_movement(&self, orientation: Quat) -> MovementGlam {
        match self {
            PidAxis::Depth => MovementGlam {
                force: orientation.inverse() * Vec3A::NEG_Z,
                torque: Vec3A::ZERO,
            },
            PidAxis::Yaw => MovementGlam {
                force: Vec3A::ZERO,
                torque: Vec3A::Z,
            },
            PidAxis::Pitch => MovementGlam {
                force: Vec3A::ZERO,
                torque: Vec3A::X,
            },
            PidAxis::Roll => MovementGlam {
                force: Vec3A::ZERO,
                torque: Vec3A::Y,
            },
        }
    }

    fn get_unit_global_movement(&self, orientation: Quat) -> MovementGlam {
        match self {
            PidAxis::Depth => MovementGlam {
                force: Vec3A::NEG_Z,
                torque: Vec3A::ZERO,
            },
            PidAxis::Yaw => MovementGlam {
                force: Vec3A::ZERO,
                torque: orientation * Vec3A::Z,
            },
            PidAxis::Pitch => MovementGlam {
                force: Vec3A::ZERO,
                torque: orientation * Vec3A::X,
            },
            PidAxis::Roll => MovementGlam {
                force: Vec3A::ZERO,
                torque: orientation * Vec3A::Y,
            },
        }
    }
}

fn setup_stabalize(mut cmds: Commands, robot: Res<LocalRobot>, config: Res<RobotConfig>) {
    for (axis, pid_config) in &config.pid_configs {
        cmds.spawn((
            MovementContributionBundle {
                name: Name::new(format!("Stabalize {axis:?}")),
                contribution: MovementContribution(MovementGlam::default()),
                robot: RobotId(robot.net_id),
            },
            pid_config.clone(),
            *axis,
            PidState::default(),
            Replicate,
        ));
    }
}

fn stabalize_system(
    mut cmds: Commands,
    robot_query: Query<
        (
            &Armed,
            Option<&Orientation>,
            Option<&OrientationTarget>,
            Option<&DepthMeasurement>,
            Option<&DepthTarget>,
        ),
        With<LocalRobotMarker>,
    >,
    mut conntroller_query: Query<(Entity, &PidConfig, &PidAxis, &mut PidState)>,
    time: Res<Time<Real>>,
) {
    let (armed, orientation, orientation_target, depth, depth_target) = robot_query.single();

    let mut orientation_error = orientation_target
        .zip(orientation)
        .map(|(orientation_target, orientation)| orientation_target.0 * orientation.0.inverse());
    let mut depth_error = depth_target
        .zip(depth)
        .map(|(depth_target, depth)| depth_target.0 - depth.depth);

    if *armed != Armed::Armed {
        orientation_error = None;
        depth_error = None;
    }

    for (entity, config, axis, mut state) in conntroller_query.iter_mut() {
        let needs_remove = 'pid_result: {
            let Some(orientation) = orientation else {
                break 'pid_result true;
            };

            let res = match axis {
                PidAxis::Depth => depth_error
                    .map(|depth_error| state.0.update(depth_error.0, config, time.delta())),
                PidAxis::Yaw | PidAxis::Pitch | PidAxis::Roll => {
                    orientation_error.map(|orientation_error| {
                        let error = instant_twist(
                            orientation_error,
                            axis.get_unit_global_movement(orientation.0).torque,
                        )
                        .to_degrees();

                        state.0.update(error, config, time.delta())
                    })
                }
            };
            if let Some(res) = res {
                let movement = axis.get_unit_local_movement(orientation.0) * res.correction;
                cmds.entity(entity)
                    .insert((MovementContribution(movement), res));
                false
            } else {
                true
            }
        };

        if needs_remove {
            cmds.entity(entity)
                .remove::<(MovementContribution, PidResult)>();

            state.0.reset_i();
        }
    }
}
fn instant_twist(q: Quat, twist_axis: Vec3A) -> f32 {
    let rotation_axis = vec3a(q.x, q.y, q.z);

    let sign = rotation_axis.dot(twist_axis).signum();
    let projected = rotation_axis.project_onto(twist_axis);
    let twist = Quat::from_xyzw(projected.x, projected.y, projected.z, q.w).normalize() * sign;

    let angle = twist.w.acos() * 2.0;
    normalize_angle(angle)
}

fn normalize_angle(angle: f32) -> f32 {
    let wrapped_angle = modf(angle, TAU);
    if wrapped_angle > PI {
        wrapped_angle - TAU
    } else {
        wrapped_angle
    }
}

fn modf(a: f32, b: f32) -> f32 {
    (a % b + b) % b
}
