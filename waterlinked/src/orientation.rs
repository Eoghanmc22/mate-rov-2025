use core::f32;
use std::time::Duration;

use bevy::prelude::*;
use common::components::OrientationTarget;

pub struct SpinPlugin;

impl Plugin for SpinPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, orientation_controller);
    }
}

#[derive(Component)]
pub struct OrientationState {
    start_time: Duration,
}

impl FromWorld for OrientationState {
    fn from_world(world: &mut World) -> Self {
        Self {
            start_time: world.resource::<Time<Real>>().elapsed(),
        }
    }
}

fn orientation_controller(
    mut cmds: Commands,
    query: Query<(Entity, &OrientationState)>,
    time: Res<Time<Real>>,
) {
    for (entity, state) in &query {
        let elapsed = time.elapsed() - state.start_time;
        cmds.entity(entity)
            .insert(OrientationTarget(get_quat_after(elapsed)));
    }
}

// fn get_quat_after(time: Duration) -> Quat {
//     // Quat::from_axis_angle(Vec3::new(1.0, 1.0, 1.0).normalize(), time.as_secs_f32())
//     Quat::from_euler(EulerRot::ZYX, time.as_secs_f32(), 0.0, 0.0)
//     // Quat::from_euler(EulerRot::YZX, time.as_secs_f32(), 0.0, 0.0)
// }

fn get_quat_after(time: Duration) -> Quat {
    // Yaw
    Quat::from_euler(
        EulerRot::ZYX,
        time.as_secs_f32().sin() * f32::consts::PI * 1.5,
        0.0,
        0.0,
    )
    // Roll
    // Quat::from_euler(EulerRot::ZYX, 0.0, time.as_secs_f32().sin() * f32::consts::PI * 0.5, 0.0)
    // Pitch
    // Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, time.as_secs_f32().sin() * f32::consts::PI * 0.5)
}
