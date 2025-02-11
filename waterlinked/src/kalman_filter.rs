// pub mod filter;
pub mod simple_filter;

use bevy::prelude::*;
use common::components::{Robot, RobotId};
// use minikalman::extended::builder::KalmanFilterBuilder;
//
// minikalman: lib seems broken
// adskalman: works but doesnt support ekf (maybe not needed?)
// kfilter: need to look into
// bayes_estimate: need to look into

use crate::trajectory::RawPose;

// Thoughts:
// position: (x, y, z)
// velocity: (x, y, z)
// acceleration: (x, y, z)
// Angle: (x, y, z)
// Angle velo: (x, y, z)
// Angle accel: (x, y, z)
// thruster force: (x, y, z)
// thruster torque: (x, y, z)
// linear drag: (x, y, z)
// angular drag: (x, y, z)
// mass
// center of mass: (x, y, z)
// accelerometer bias: (x, y, z)
// gyro bias: (x, y, z)

// Position: (x, y, z)
// Velocity: (x, y, z)
// Orientation Quat: (w, x, y, z)
// Angular Velocity: (x, y, z)
// Accelerometer Bias: (x, y, z)
// const NUM_STATES: usize = 16;

// Thruster Force: (x, y, z)
// Thruster Torque: (x, y, z)
// const NUM_CONTROLS: usize = 6;

// IMU Accel: (x, y, z)
// Depth: (z)
// UGPS: (x, y, z)
// Orientation: (w, x, y, z)
// const NUM_OBSERVATIONS: usize = 11;

pub struct KalmanPlugin;

impl Plugin for KalmanPlugin {
    fn build(&self, app: &mut App) {
        // let builder = KalmanFilterBuilder::<NUM_STATES, f32>::default();
        // let mut filter = builder.build();
        // let mut measurement = builder.observations().build::<NUM_OBSERVATIONS>();

        //
    }
}

#[derive(Component)]
struct KalmanFilterComponent {
    // TODO
}

fn kalman_filter(
    mut cmds: Commands,
    robot: Query<(&RawPose, &KalmanFilterComponent, &RobotId), With<Robot>>,
) {
    //
}
