use core::f32;
use std::time::Duration;

use bevy::{
    ecs::{component::ComponentId, world::DeferredWorld},
    math::Vec3A,
    prelude::*,
};
use common::{
    bundles::MovementContributionBundle,
    components::{MovementContribution, Orientation, OrientationTarget},
};
use motor_math::glam::MovementGlam;

pub struct SpinPlugin;

impl Plugin for SpinPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, orientation_controller);
    }
}

#[derive(Component)]
#[component(on_insert = insert_orientation_state, on_replace = replace_orientation_state)]
pub struct OrientationState {
    start_time: Duration,
    feedforward: Entity,
}

impl FromWorld for OrientationState {
    fn from_world(world: &mut World) -> Self {
        Self {
            start_time: world.resource::<Time<Real>>().elapsed(),
            feedforward: Entity::PLACEHOLDER,
        }
    }
}

fn insert_orientation_state(mut world: DeferredWorld, entity: Entity, _component: ComponentId) {
    let robot = *world.get(entity).unwrap();
    let contributor = world
        .commands()
        .spawn(MovementContributionBundle {
            name: Name::new("Orientation Feedforward"),
            contribution: MovementContribution(MovementGlam::default()),
            robot,
        })
        .id();

    world
        .get_mut::<OrientationState>(entity)
        .unwrap()
        .feedforward = contributor;
}

fn replace_orientation_state(mut world: DeferredWorld, entity: Entity, _component: ComponentId) {
    let contributor = world.get::<OrientationState>(entity).unwrap().feedforward;
    world.commands().entity(contributor).despawn();
}

fn orientation_controller(
    mut cmds: Commands,
    query: Query<(Entity, &Orientation, &OrientationState)>,
    time: Res<Time<Real>>,
) {
    for (entity, orientation, state) in &query {
        let elapsed = time.elapsed() - state.start_time;
        let (target_quat, movement) = get_control_output(elapsed, orientation.0);

        cmds.entity(entity).insert(OrientationTarget(target_quat));
        cmds.entity(state.feedforward)
            .insert(MovementContribution(movement));
    }
}

// fn get_quat_after(time: Duration) -> Quat {
//     // Quat::from_axis_angle(Vec3::new(1.0, 1.0, 1.0).normalize(), time.as_secs_f32())
//     Quat::from_euler(EulerRot::ZYX, time.as_secs_f32(), 0.0, 0.0)
//     // Quat::from_euler(EulerRot::YZX, time.as_secs_f32(), 0.0, 0.0)
// }

fn get_control_output(time: Duration, current: Quat) -> (Quat, MovementGlam) {
    // // Yaw
    // Quat::from_euler(
    //     EulerRot::ZYX,
    //     (time.as_secs_f32() / 10.0).sin() * f32::consts::PI * 1.5,
    //     0.0,
    //     0.0,
    // )

    let mut yaw = current;
    if yaw.z.abs() * yaw.z.abs() + yaw.w.abs() * yaw.w.abs() > 0.1 {
        yaw.x = 0.0;
        yaw.y = 0.0;
        yaw = yaw.normalize()
    } else {
        yaw *= Quat::from_rotation_y(180f32.to_radians());
        yaw.x = 0.0;
        yaw.y = 0.0;
        yaw = -yaw.normalize();
        // yaw *= Quat::from_rotation_y(180f32.to_radians()).inverse();
    }

    // Roll
    let roll = (time.as_secs_f32() / 10.0).sin() * f32::consts::PI * 0.25;
    (
        yaw * Quat::from_euler(EulerRot::ZYX, 0.0, roll, 0.0),
        MovementGlam {
            force: Vec3A::ZERO,
            torque: Vec3A::new(0.0, roll * 0.5, 0.0),
        },
    )
    // // Pitch
    // Quat::from_euler(
    //     EulerRot::ZYX,
    //     0.0,
    //     0.0,
    //     (time.as_secs_f32() / 10.0).sin() * f32::consts::PI * 0.25,
    // )
}
