use bevy::{
    app::{Plugin, Update},
    core::Name,
    math::{NormedVectorSpace, Quat, Vec3A},
    prelude::{App, Commands, Component, Entity, Local, Query, Res, With},
};
use common::{
    bundles::MovementContributionBundle,
    components::{FilteredPose, MovementContribution, Pose, Robot, RobotId, TargetPose},
    ecs_sync::Replicate,
};
use motor_math::glam::MovementGlam;

use crate::waterlinked::WaterlinkedAngleOffset;

pub const FORCE_GAIN: f32 = 15.0;
pub const TORQUE_GAIN: f32 = 0.5;
pub const MAX_FORCE: f32 = 25.0;

pub struct TrajectoryPlugin;

impl Plugin for TrajectoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, trajectory_follower);
    }
}

// NOTE: Outputs are unscaled
pub fn move_toward(current_pose: &Pose, target_pose: &Pose) -> MovementGlam {
    let mut translation =
        current_pose.rotation.inverse() * (target_pose.position - current_pose.position);
    let mut rotation = target_pose.rotation * current_pose.rotation.inverse();

    // FIXME: Temp simplification to prevent fighting with the other control systems
    translation.z = 0.0;
    rotation = Quat::IDENTITY;

    MovementGlam {
        force: translation,
        torque: rotation.to_scaled_axis().into(),
    }
}

// FIXME: Ideally, this would run on the rov
fn trajectory_follower(
    mut movement_contributer: Local<Option<Entity>>,

    mut cmds: Commands,
    robot: Query<(&FilteredPose, &TargetPose, &RobotId), With<Robot>>,
) {
    let Ok((current_pose, target_pose, robot_id)) = robot.get_single() else {
        if let Some(entity) = *movement_contributer {
            cmds.entity(entity).despawn();
            *movement_contributer = None;
        }

        return;
    };

    let mut movement = move_toward(&current_pose.pose, &target_pose.0);
    movement.force = movement.force;
    movement.force *= FORCE_GAIN;
    movement.torque *= TORQUE_GAIN;

    if movement.force.norm_squared() > MAX_FORCE * MAX_FORCE {
        movement.force = movement.force.normalize() * MAX_FORCE;
    }

    println!("movement: {movement:.2?}");

    if let Some(entity) = *movement_contributer {
        cmds.entity(entity).insert(MovementContribution(movement));
    } else {
        let entity = cmds
            .spawn((
                MovementContributionBundle {
                    name: Name::new("Trajectory Follower"),
                    contribution: MovementContribution(movement),
                    robot: *robot_id,
                },
                Replicate,
            ))
            .id();
        *movement_contributer = Some(entity);
    }
}
