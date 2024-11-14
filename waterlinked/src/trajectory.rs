use bevy::{
    app::{Plugin, Update},
    core::Name,
    math::{Quat, Vec3A},
    prelude::{App, Commands, Component, Entity, Local, Query, With},
};
use common::{
    bundles::MovementContributionBundle,
    components::{MovementContribution, Robot, RobotId},
};
use motor_math::glam::MovementGlam;

pub const FORCE_GAIN: f32 = 0.01;
pub const TORQUE_GAIN: f32 = 0.5;

pub struct TrajectoryPlugin;

impl Plugin for TrajectoryPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, trajectory_follower);
    }
}

// Consider using Isometry3d in bevy 15
#[derive(Debug)]
pub struct Pose {
    pub position: Vec3A,
    pub rotation: Quat,
}

#[derive(Component, Debug)]
pub struct TargetPose(pub Pose);

#[derive(Component, Debug)]
pub struct CurrentPose(pub Pose);

// NOTE: Outputs are unscaled
pub fn move_toward(current_pose: &Pose, target_pose: &Pose) -> MovementGlam {
    let translation =
        current_pose.rotation.inverse() * (target_pose.position - current_pose.position);
    let rotation = target_pose.rotation * current_pose.rotation.inverse();

    MovementGlam {
        force: translation,
        torque: rotation.to_scaled_axis().into(),
    }
}

// FIXME: Ideally, this would run on the rov
fn trajectory_follower(
    mut movement_contributer: Local<Option<Entity>>,

    mut cmds: Commands,
    robot: Query<(&CurrentPose, &TargetPose, &RobotId), With<Robot>>,
) {
    let Ok((current_pose, target_pose, robot_id)) = robot.get_single() else {
        if let Some(entity) = *movement_contributer {
            cmds.entity(entity).despawn();
            *movement_contributer = None;
        }

        return;
    };

    let mut movement = move_toward(&current_pose.0, &target_pose.0);
    movement.force *= FORCE_GAIN;
    movement.torque *= TORQUE_GAIN;

    if let Some(entity) = *movement_contributer {
        cmds.entity(entity).insert(MovementContribution(movement));
    } else {
        cmds.spawn(MovementContributionBundle {
            name: Name::new("Trajectory Follower"),
            contribution: MovementContribution(movement),
            robot: *robot_id,
        });
    }
}
