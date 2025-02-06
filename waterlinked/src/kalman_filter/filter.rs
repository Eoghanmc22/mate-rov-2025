use bevy::math::{Quat, Vec3, Vec3A};
use minikalman::extended::builder::KalmanFilterBuilder;
use minikalman::extended::builder::{KalmanFilterObservationType, KalmanFilterType};
use minikalman::prelude::*;

// Constants for state indices
// X, Y, Z are world space axis
pub mod constants {
    pub const NUM_STATES: usize = 22;
    pub const POS_X: usize = 0;
    pub const POS_Y: usize = 1;
    pub const POS_Z: usize = 2;
    pub const VEL_X: usize = 3;
    pub const VEL_Y: usize = 4;
    pub const VEL_Z: usize = 5;
    // Without Gravity
    pub const ACC_X: usize = 6;
    pub const ACC_Y: usize = 7;
    pub const ACC_Z: usize = 8;
    pub const QUAT_W: usize = 9;
    pub const QUAT_X: usize = 10;
    pub const QUAT_Y: usize = 11;
    pub const QUAT_Z: usize = 12;
    // These are in local space
    pub const ANGVEL_X: usize = 13;
    pub const ANGVEL_Y: usize = 14;
    pub const ANGVEL_Z: usize = 15;
    pub const BIAS_ACC_X: usize = 16;
    pub const BIAS_ACC_Y: usize = 17;
    pub const BIAS_ACC_Z: usize = 18;
    pub const BIAS_GYRO_X: usize = 19;
    pub const BIAS_GYRO_Y: usize = 20;
    pub const BIAS_GYRO_Z: usize = 21;
}

// Constants for measurement indices
pub mod meas_constants {
    pub const NUM_OBSERVATIONS: usize = 14;
    // World space
    pub const DEPTH: usize = 0;
    pub const UPOS_X: usize = 1;
    pub const UPOS_Y: usize = 2;
    pub const UPOS_Z: usize = 3;
    // These are in local space
    pub const ACC_X: usize = 4;
    pub const ACC_Y: usize = 5;
    pub const ACC_Z: usize = 6;
    pub const GYRO_X: usize = 7;
    pub const GYRO_Y: usize = 8;
    pub const GYRO_Z: usize = 9;
    pub const QUAT_W: usize = 10;
    pub const QUAT_X: usize = 11;
    pub const QUAT_Y: usize = 12;
    pub const QUAT_Z: usize = 13;
}

pub struct Measurement {
    depth: Option<f32>,
    upos: Option<Vec3A>,
    acc: Option<Vec3A>,
    gyro_raw: Option<Vec3A>,
    orientation: Option<Quat>,
}

/// Configuration parameters for the ROV.
#[derive(Debug)]
pub struct RovConfig {
    pub process_noise: f32,
    pub depth_noise: f32,
    pub upos_noise: f32,
    pub acc_noise: f32,
    pub gyro_noise: f32,
    pub orientation_noise: f32,
}

type Filter = KalmanFilterType<{ constants::NUM_STATES }, f32>;
type Observation = KalmanFilterObservationType<
    { constants::NUM_STATES },
    { meas_constants::NUM_OBSERVATIONS },
    f32,
>;

/// Manager for the Extended Kalman Filter.
pub struct EkfManager {
    filter: Filter,
    measurement: Observation,
    config: RovConfig,
}

impl EkfManager {
    /// Creates a new EKF manager with the given configuration.
    pub fn new(config: RovConfig) -> Self {
        let builder = KalmanFilterBuilder::<{ constants::NUM_STATES }, f32>::default();
        let mut filter = builder.build();
        let mut measurement = builder
            .observations()
            .build::<{ meas_constants::NUM_OBSERVATIONS }>();

        // Initialize state vector
        initialize_state_vector(&mut filter);

        // Initialize covariance matrix with appropriate uncertainties
        initialize_estimate_covariance(&mut filter);

        // Initialize process noise with different variances per state variable
        initialize_process_noise(&mut filter, &config);

        // Initialize measurement noise with different variances per measurement
        initialize_measurement_noise(&mut measurement, &config);

        Self {
            filter,
            measurement,
            config,
        }
    }

    /// Predicts the state at the next time step.
    pub fn predict(&mut self, delta_t: f32) {
        self.update_state_transition_jacobian(delta_t);
        self.filter.predict_nonlinear(|state, next| {
            // Position update
            next.set_row(
                constants::POS_X,
                state.get_row(constants::POS_X) + state.get_row(constants::VEL_X) * delta_t,
            );
            next.set_row(
                constants::POS_Y,
                state.get_row(constants::POS_Y) + state.get_row(constants::VEL_Y) * delta_t,
            );
            next.set_row(
                constants::POS_Z,
                state.get_row(constants::POS_Z) + state.get_row(constants::VEL_Z) * delta_t,
            );

            // Velocity update
            next.set_row(
                constants::VEL_X,
                state.get_row(constants::VEL_X) + state.get_row(constants::ACC_X) * delta_t,
            );
            next.set_row(
                constants::VEL_Y,
                state.get_row(constants::VEL_Y) + state.get_row(constants::ACC_Y) * delta_t,
            );
            next.set_row(
                constants::VEL_Z,
                state.get_row(constants::VEL_Z) + state.get_row(constants::ACC_Z) * delta_t,
            );

            // Assume acceleration is unchanged
            next.set_row(constants::ACC_X, state.get_row(constants::ACC_X));
            next.set_row(constants::ACC_Y, state.get_row(constants::ACC_Y));
            next.set_row(constants::ACC_Z, state.get_row(constants::ACC_Z));

            // Orientation update using quaternion multiplication
            let q = Quat::from_xyzw(
                state.get_row(constants::QUAT_X),
                state.get_row(constants::QUAT_Y),
                state.get_row(constants::QUAT_Z),
                state.get_row(constants::QUAT_W),
            )
            .normalize();

            let omega = Vec3::new(
                state.get_row(constants::ANGVEL_X),
                state.get_row(constants::ANGVEL_Y),
                state.get_row(constants::ANGVEL_Z),
            );
            let delta_q = Quat::from_scaled_axis(omega * delta_t);
            let new_q = q * delta_q;

            next.set_row(constants::QUAT_W, new_q.w);
            next.set_row(constants::QUAT_X, new_q.x);
            next.set_row(constants::QUAT_Y, new_q.y);
            next.set_row(constants::QUAT_Z, new_q.z);

            // Normalize the quaternion to prevent drift
            let normalized_q = new_q.normalize();
            next.set_row(constants::QUAT_W, normalized_q.w);
            next.set_row(constants::QUAT_X, normalized_q.x);
            next.set_row(constants::QUAT_Y, normalized_q.y);
            next.set_row(constants::QUAT_Z, normalized_q.z);

            // Assume angular velocity is unchanged
            next.set_row(constants::ANGVEL_X, state.get_row(constants::ANGVEL_X));
            next.set_row(constants::ANGVEL_Y, state.get_row(constants::ANGVEL_Y));
            next.set_row(constants::ANGVEL_Z, state.get_row(constants::ANGVEL_Z));

            // Bias terms (assuming random walk model)
            next.set_row(constants::BIAS_ACC_X, state.get_row(constants::BIAS_ACC_X));
            next.set_row(constants::BIAS_ACC_Y, state.get_row(constants::BIAS_ACC_Y));
            next.set_row(constants::BIAS_ACC_Z, state.get_row(constants::BIAS_ACC_Z));
            next.set_row(
                constants::BIAS_GYRO_X,
                state.get_row(constants::BIAS_GYRO_X),
            );
            next.set_row(
                constants::BIAS_GYRO_Y,
                state.get_row(constants::BIAS_GYRO_Y),
            );
            next.set_row(
                constants::BIAS_GYRO_Z,
                state.get_row(constants::BIAS_GYRO_Z),
            );
        });
    }

    /// Updates the state transition Jacobian matrix.
    fn update_state_transition_jacobian(&mut self, delta_t: f32) {
        // Orientation (quaternion) derivatives
        // Using quaternion dynamics: q_dot = 0.5 * q * omega
        let omega_x = self.filter.state_vector().get_row(constants::ANGVEL_X);
        let omega_y = self.filter.state_vector().get_row(constants::ANGVEL_Y);
        let omega_z = self.filter.state_vector().get_row(constants::ANGVEL_Z);

        self.filter.state_transition_jacobian_mut().apply(|mat| {
            // Clear the matrix
            mat.clear();

            // Position derivatives with respect to position and velocity
            for i in 0..3 {
                mat.set(constants::POS_X + i, constants::POS_X + i, 1.0);
                mat.set(constants::POS_X + i, constants::VEL_X + i, delta_t);
            }

            // Velocity derivatives with respect to velocity and acceleration
            for i in 0..3 {
                mat.set(constants::VEL_X + i, constants::VEL_X + i, 1.0);
                mat.set(constants::VEL_X + i, constants::ACC_X + i, delta_t);
            }

            // Acceleration derivatives (assuming constant acceleration)
            for i in 0..3 {
                mat.set(constants::ACC_X + i, constants::ACC_X + i, 1.0);
            }

            // Compute the Jacobian for quaternion dynamics
            // Partial derivatives based on q_dot = 0.5 * q * omega * delta_t
            let half_dt = 0.5 * delta_t;
            mat.set(constants::QUAT_W, constants::QUAT_W, 1.0);
            mat.set(constants::QUAT_W, constants::QUAT_X, -half_dt * omega_x);
            mat.set(constants::QUAT_W, constants::QUAT_Y, -half_dt * omega_y);
            mat.set(constants::QUAT_W, constants::QUAT_Z, -half_dt * omega_z);
            mat.set(constants::QUAT_X, constants::QUAT_W, half_dt * omega_x);
            mat.set(constants::QUAT_X, constants::QUAT_X, 1.0);
            mat.set(constants::QUAT_X, constants::QUAT_Y, half_dt * omega_z);
            mat.set(constants::QUAT_X, constants::QUAT_Z, -half_dt * omega_y);
            mat.set(constants::QUAT_Y, constants::QUAT_W, half_dt * omega_y);
            mat.set(constants::QUAT_Y, constants::QUAT_X, -half_dt * omega_z);
            mat.set(constants::QUAT_Y, constants::QUAT_Y, 1.0);
            mat.set(constants::QUAT_Y, constants::QUAT_Z, half_dt * omega_x);
            mat.set(constants::QUAT_Z, constants::QUAT_W, half_dt * omega_z);
            mat.set(constants::QUAT_Z, constants::QUAT_X, half_dt * omega_y);
            mat.set(constants::QUAT_Z, constants::QUAT_Y, -half_dt * omega_x);
            mat.set(constants::QUAT_Z, constants::QUAT_Z, 1.0);

            // Angular velocity derivatives (assuming constant angular velocity)
            for i in 0..3 {
                mat.set(constants::ANGVEL_X + i, constants::ANGVEL_X + i, 1.0);
            }

            // Bias terms Jacobian (assuming constant biases)
            for i in 0..3 {
                mat.set(constants::BIAS_ACC_X + i, constants::BIAS_ACC_X + i, 1.0);
                mat.set(constants::BIAS_GYRO_X + i, constants::BIAS_GYRO_X + i, 1.0);
            }
        });
    }

    /// Corrects the state based on the current measurement.
    pub fn correct(&mut self) {
        self.filter
            .correct_nonlinear(&mut self.measurement, |state, observation| {
                let q = Quat::from_xyzw(
                    state.get_row(constants::QUAT_X),
                    state.get_row(constants::QUAT_Y),
                    state.get_row(constants::QUAT_Z),
                    state.get_row(constants::QUAT_W),
                )
                .normalize();
                let gravity_world = Vec3A::new(0.0, 0.0, 9.81);
                let gravity_body = q.inverse() * gravity_world;

                // Transform the state into an observation.
                observation[meas_constants::DEPTH] = state.get_row(constants::POS_Z);
                observation[meas_constants::UPOS_X] = state.get_row(constants::POS_X);
                observation[meas_constants::UPOS_Y] = state.get_row(constants::POS_Y);
                observation[meas_constants::UPOS_Z] = state.get_row(constants::POS_Z);

                // Accelerometer measures: acc = true_acc + bias_acc + gravity_body
                observation[meas_constants::ACC_X] = state.get_row(constants::ACC_X)
                    + state.get_row(constants::BIAS_ACC_X)
                    + gravity_body.x;
                observation[meas_constants::ACC_Y] = state.get_row(constants::ACC_Y)
                    + state.get_row(constants::BIAS_ACC_Y)
                    + gravity_body.y;
                observation[meas_constants::ACC_Z] = state.get_row(constants::ACC_Z)
                    + state.get_row(constants::BIAS_ACC_Z)
                    + gravity_body.z;

                // Gyroscope measures: gyro = angular_velocity + bias_gyro
                observation[meas_constants::GYRO_X] =
                    state.get_row(constants::ANGVEL_X) + state.get_row(constants::BIAS_GYRO_X);
                observation[meas_constants::GYRO_Y] =
                    state.get_row(constants::ANGVEL_Y) + state.get_row(constants::BIAS_GYRO_Y);
                observation[meas_constants::GYRO_Z] =
                    state.get_row(constants::ANGVEL_Z) + state.get_row(constants::BIAS_GYRO_Z);

                // Orientation measurement directly observes the quaternion
                observation[meas_constants::QUAT_W] = q.w;
                observation[meas_constants::QUAT_X] = q.x;
                observation[meas_constants::QUAT_Y] = q.y;
                observation[meas_constants::QUAT_Z] = q.z;
            });
    }

    /// Handles partial measurements by updating only the available measurements.
    pub fn update_measurements(&mut self, observation_input: Measurement) {
        self.update_observation_jacobian(&observation_input);
        self.measurement
            .measurement_vector_mut()
            .apply(|measurement| {
                // Reset measurement vector to zero before applying new measurements
                measurement.clear();

                if let Some(d) = observation_input.depth {
                    measurement.set_row(meas_constants::DEPTH, d);
                }
                if let Some(p) = observation_input.upos {
                    measurement.set_row(meas_constants::UPOS_X, p.x);
                    measurement.set_row(meas_constants::UPOS_Y, p.y);
                    measurement.set_row(meas_constants::UPOS_Z, p.z);
                }
                if let Some(a) = observation_input.acc {
                    measurement.set_row(meas_constants::ACC_X, a.x);
                    measurement.set_row(meas_constants::ACC_Y, a.y);
                    measurement.set_row(meas_constants::ACC_Z, a.z);
                }
                if let Some(g) = observation_input.gyro_raw {
                    measurement.set_row(meas_constants::GYRO_X, g.x);
                    measurement.set_row(meas_constants::GYRO_Y, g.y);
                    measurement.set_row(meas_constants::GYRO_Z, g.z);
                }
                if let Some(o) = observation_input.orientation {
                    measurement.set_row(meas_constants::QUAT_W, o.w);
                    measurement.set_row(meas_constants::QUAT_X, o.x);
                    measurement.set_row(meas_constants::QUAT_Y, o.y);
                    measurement.set_row(meas_constants::QUAT_Z, o.z);
                }
            });
        self.correct();
    }

    //     /// Updates the observation Jacobian matrix.
    //     fn update_observation_jacobian(&mut self, observation_input: &Measurement) {
    //         self.measurement
    //             .observation_jacobian_matrix_mut()
    //             .apply(|mat| {
    //                 mat.clear();
    //
    //                 // Depth measurement (z position)
    //                 if observation_input.depth.is_some() {
    //                     mat.set(meas_constants::DEPTH, constants::POS_Z, 1.0);
    //                 }
    //
    //                 // UPOS measurement (position)
    //                 if observation_input.upos.is_some() {
    //                     mat.set(meas_constants::UPOS_X, constants::POS_X, 1.0);
    //                     mat.set(meas_constants::UPOS_Y, constants::POS_Y, 1.0);
    //                     mat.set(meas_constants::UPOS_Z, constants::POS_Z, 1.0);
    //                 }
    //
    //                 // Accelerometer measurement
    //                 if observation_input.acc.is_some() {
    //                     // Accelerometer measures: acc = true_acc + bias_acc + gravity_body
    //                     mat.set(meas_constants::ACC_X, constants::ACC_X, 1.0);
    //                     mat.set(meas_constants::ACC_Y, constants::ACC_Y, 1.0);
    //                     mat.set(meas_constants::ACC_Z, constants::ACC_Z, 1.0);
    //
    //                     mat.set(meas_constants::ACC_X, constants::BIAS_ACC_X, 1.0);
    //                     mat.set(meas_constants::ACC_Y, constants::BIAS_ACC_Y, 1.0);
    //                     mat.set(meas_constants::ACC_Z, constants::BIAS_ACC_Z, 1.0);
    //
    //                     // Compute partial derivatives of gravity_body with respect to quaternion
    //                     // gravity_body = q.inverse() * gravity_world
    //                     // Partial derivatives are complex; for simplicity, they are approximated here
    //                     // A more accurate implementation should derive these based on the rotation conventions
    //                     let q = Quat::from_xyzw(
    //                         self.filter.state_vector().get_row(constants::QUAT_X),
    //                         self.filter.state_vector().get_row(constants::QUAT_Y),
    //                         self.filter.state_vector().get_row(constants::QUAT_Z),
    //                         self.filter.state_vector().get_row(constants::QUAT_W),
    //                     )
    //                     .normalize();
    //
    //                     let gravity_world = Vec3A::new(0.0, 0.0, 9.81);
    //                     // Partial derivatives of gravity_body w.r. to quaternion components
    //                     // Using numerical approximation or analytical derivatives
    //                     // Here, an approximate linear relationship is assumed
    //                     // Consider implementing exact derivatives for better accuracy
    //
    //                     // Example partial derivatives (refined)
    //                     let q_w = q.w;
    //                     let q_x = q.x;
    //                     let q_y = q.y;
    //                     let q_z = q.z;
    //
    //                     // Partial derivatives of gravity_body with respect to quaternion components
    //                     // Using the formula for quaternion inverse and multiplication
    //                     // gravity_body = q.inverse() * gravity_world * q
    //                     // Simplified partial derivatives based on standard quaternion derivative formulas
    //
    //                     let dg_dqw = Vec3A::new(
    //                         -2.0 * gravity_world.x * q_y + 2.0 * gravity_world.y * q_x,
    //                         -2.0 * gravity_world.x * q_z + 2.0 * gravity_world.z * q_x,
    //                         2.0 * gravity_world.x * q_y - 2.0 * gravity_world.y * q_x,
    //                     );
    //
    //                     let dg_dqx = Vec3A::new(
    //                         2.0 * gravity_world.x * q_w + 2.0 * gravity_world.y * q_z
    //                             - 2.0 * gravity_world.z * q_y,
    //                         2.0 * gravity_world.x * q_z
    //                             - 2.0 * gravity_world.y * q_w
    //                             - 2.0 * gravity_world.z * q_x,
    //                         2.0 * gravity_world.y * q_w + 2.0 * gravity_world.z * q_x
    //                             - 2.0 * gravity_world.x * q_y,
    //                     );
    //
    //                     let dg_dqy = Vec3A::new(
    //                         2.0 * gravity_world.x * q_z
    //                             - 2.0 * gravity_world.w * q_x
    //                             - 2.0 * gravity_world.z * q_w,
    //                         2.0 * gravity_world.x * q_w + 2.0 * gravity_world.z * q_x
    //                             - 2.0 * gravity_world.y * q_z,
    //                         2.0 * gravity_world.w * q_x - 2.0 * gravity_world.y * q_z
    //                             + 2.0 * gravity_world.z * q_w,
    //                     );
    //
    //                     let dg_dqz = Vec3A::new(
    //                         2.0 * gravity_world.x * q_y - 2.0 * gravity_world.w * q_x
    //                             + 2.0 * gravity_world.z * q_w,
    //                         -2.0 * gravity_world.x * q_x
    //                             + 2.0 * gravity_world.w * q_z
    //                             + 2.0 * gravity_world.y * q_w,
    //                         2.0 * gravity_world.w * q_z
    //                             - 2.0 * gravity_world.y * q_x
    //                             - 2.0 * gravity_world.x * q_w,
    //                     );
    //
    //                     // Set partial derivatives in the Jacobian matrix
    //                     mat.set(meas_constants::ACC_X, constants::QUAT_W, dg_dqw.x);
    //                     mat.set(meas_constants::ACC_X, constants::QUAT_X, dg_dqx.x);
    //                     mat.set(meas_constants::ACC_X, constants::QUAT_Y, dg_dqy.x);
    //                     mat.set(meas_constants::ACC_X, constants::QUAT_Z, dg_dqz.x);
    //
    //                     mat.set(meas_constants::ACC_Y, constants::QUAT_W, dg_dqw.y);
    //                     mat.set(meas_constants::ACC_Y, constants::QUAT_X, dg_dqx.y);
    //                     mat.set(meas_constants::ACC_Y, constants::QUAT_Y, dg_dqy.y);
    //                     mat.set(meas_constants::ACC_Y, constants::QUAT_Z, dg_dqz.y);
    //
    //                     mat.set(meas_constants::ACC_Z, constants::QUAT_W, dg_dqw.z);
    //                     mat.set(meas_constants::ACC_Z, constants::QUAT_X, dg_dqx.z);
    //                     mat.set(meas_constants::ACC_Z, constants::QUAT_Y, dg_dqy.z);
    //                     mat.set(meas_constants::ACC_Z, constants::QUAT_Z, dg_dqz.z);
    //                 }
    //
    //                 // Gyro measurement
    //                 if observation_input.gyro_raw.is_some() {
    //                     mat.set(meas_constants::GYRO_X, constants::ANGVEL_X, 1.0);
    //                     mat.set(meas_constants::GYRO_Y, constants::ANGVEL_Y, 1.0);
    //                     mat.set(meas_constants::GYRO_Z, constants::ANGVEL_Z, 1.0);
    //
    //                     mat.set(meas_constants::GYRO_X, constants::BIAS_GYRO_X, 1.0);
    //                     mat.set(meas_constants::GYRO_Y, constants::BIAS_GYRO_Y, 1.0);
    //                     mat.set(meas_constants::GYRO_Z, constants::BIAS_GYRO_Z, 1.0);
    //                 }
    //
    //                 // Orientation measurement (quaternion)
    //                 if observation_input.orientation.is_some() {
    //                     mat.set(meas_constants::QUAT_W, constants::QUAT_W, 1.0);
    //                     mat.set(meas_constants::QUAT_X, constants::QUAT_X, 1.0);
    //                     mat.set(meas_constants::QUAT_Y, constants::QUAT_Y, 1.0);
    //                     mat.set(meas_constants::QUAT_Z, constants::QUAT_Z, 1.0);
    //                 }
    //             });
    //     }
    // }
    fn update_observation_jacobian(&mut self, observation_input: &Measurement) {
        self.measurement
            .observation_jacobian_matrix_mut()
            .apply(|mat| {
                mat.clear();

                // Depth measurement (z position)
                if observation_input.depth.is_some() {
                    mat.set(meas_constants::DEPTH, constants::POS_Z, 1.0);
                }

                // UPOS measurement (position)
                if observation_input.upos.is_some() {
                    mat.set(meas_constants::UPOS_X, constants::POS_X, 1.0);
                    mat.set(meas_constants::UPOS_Y, constants::POS_Y, 1.0);
                    mat.set(meas_constants::UPOS_Z, constants::POS_Z, 1.0);
                }

                // Accelerometer measurement
                if observation_input.acc.is_some() {
                    // Accelerometer measures: acc = true_acc + bias_acc + gravity_body
                    mat.set(meas_constants::ACC_X, constants::ACC_X, 1.0);
                    mat.set(meas_constants::ACC_Y, constants::ACC_Y, 1.0);
                    mat.set(meas_constants::ACC_Z, constants::ACC_Z, 1.0);

                    mat.set(meas_constants::ACC_X, constants::BIAS_ACC_X, 1.0);
                    mat.set(meas_constants::ACC_Y, constants::BIAS_ACC_Y, 1.0);
                    mat.set(meas_constants::ACC_Z, constants::BIAS_ACC_Z, 1.0);

                    // Compute partial derivatives of gravity_body with respect to quaternion
                    let q = Quat::from_xyzw(
                        self.filter.state_vector().get_row(constants::QUAT_X),
                        self.filter.state_vector().get_row(constants::QUAT_Y),
                        self.filter.state_vector().get_row(constants::QUAT_Z),
                        self.filter.state_vector().get_row(constants::QUAT_W),
                    )
                    .normalize();

                    let gravity_world = Vec3A::new(0.0, 0.0, 9.81);
                    let g = gravity_world.z; // 9.81
                    let q_w = q.w;
                    let q_x = q.x;
                    let q_y = q.y;
                    let q_z = q.z;

                    // Partial derivatives
                    let dg_dqw = Vec3A::new(2.0 * g * q_y, -2.0 * g * q_x, 0.0);
                    let dg_dqx = Vec3A::new(-2.0 * g * q_y, -2.0 * g * q_z, -2.0 * g * q_w);
                    let dg_dqy = Vec3A::new(2.0 * g * q_w, 0.0, -2.0 * g * q_z);
                    let dg_dqz = Vec3A::new(0.0, 2.0 * g * q_w, -2.0 * g * q_y);

                    // Set partial derivatives in the Jacobian matrix
                    mat.set(meas_constants::ACC_X, constants::QUAT_W, dg_dqw.x);
                    mat.set(meas_constants::ACC_X, constants::QUAT_X, dg_dqx.x);
                    mat.set(meas_constants::ACC_X, constants::QUAT_Y, dg_dqy.x);
                    mat.set(meas_constants::ACC_X, constants::QUAT_Z, dg_dqz.x);

                    mat.set(meas_constants::ACC_Y, constants::QUAT_W, dg_dqw.y);
                    mat.set(meas_constants::ACC_Y, constants::QUAT_X, dg_dqx.y);
                    mat.set(meas_constants::ACC_Y, constants::QUAT_Y, dg_dqy.y);
                    mat.set(meas_constants::ACC_Y, constants::QUAT_Z, dg_dqz.y);

                    mat.set(meas_constants::ACC_Z, constants::QUAT_W, dg_dqw.z);
                    mat.set(meas_constants::ACC_Z, constants::QUAT_X, dg_dqx.z);
                    mat.set(meas_constants::ACC_Z, constants::QUAT_Y, dg_dqy.z);
                    mat.set(meas_constants::ACC_Z, constants::QUAT_Z, dg_dqz.z);
                }

                // Gyro measurement
                if observation_input.gyro_raw.is_some() {
                    mat.set(meas_constants::GYRO_X, constants::ANGVEL_X, 1.0);
                    mat.set(meas_constants::GYRO_Y, constants::ANGVEL_Y, 1.0);
                    mat.set(meas_constants::GYRO_Z, constants::ANGVEL_Z, 1.0);

                    mat.set(meas_constants::GYRO_X, constants::BIAS_GYRO_X, 1.0);
                    mat.set(meas_constants::GYRO_Y, constants::BIAS_GYRO_Y, 1.0);
                    mat.set(meas_constants::GYRO_Z, constants::BIAS_GYRO_Z, 1.0);
                }

                // Orientation measurement (quaternion)
                if observation_input.orientation.is_some() {
                    mat.set(meas_constants::QUAT_W, constants::QUAT_W, 1.0);
                    mat.set(meas_constants::QUAT_X, constants::QUAT_X, 1.0);
                    mat.set(meas_constants::QUAT_Y, constants::QUAT_Y, 1.0);
                    mat.set(meas_constants::QUAT_Z, constants::QUAT_Z, 1.0);
                }
            });
    }
}

/// Initializes the state vector with default values.
pub fn initialize_state_vector(filter: &mut Filter) {
    filter.state_vector_mut().apply(|state| {
        state[constants::POS_X] = 0.0;
        state[constants::POS_Y] = 0.0;
        state[constants::POS_Z] = 0.0;
        state[constants::VEL_X] = 0.0;
        state[constants::VEL_Y] = 0.0;
        state[constants::VEL_Z] = 0.0;
        state[constants::ACC_X] = 0.0;
        state[constants::ACC_Y] = 0.0;
        state[constants::ACC_Z] = 0.0;
        state[constants::QUAT_W] = 1.0;
        state[constants::QUAT_X] = 0.0;
        state[constants::QUAT_Y] = 0.0;
        state[constants::QUAT_Z] = 0.0;
        state[constants::ANGVEL_X] = 0.0;
        state[constants::ANGVEL_Y] = 0.0;
        state[constants::ANGVEL_Z] = 0.0;
        state[constants::BIAS_ACC_X] = 0.0;
        state[constants::BIAS_ACC_Y] = 0.0;
        state[constants::BIAS_ACC_Z] = 0.0;
        state[constants::BIAS_GYRO_X] = 0.0;
        state[constants::BIAS_GYRO_Y] = 0.0;
        state[constants::BIAS_GYRO_Z] = 0.0;
    });
}

/// Initializes the estimate covariance matrix with different uncertainties.
pub fn initialize_estimate_covariance(filter: &mut Filter) {
    filter.estimate_covariance_mut().apply(|mat| {
        mat.clear();
        // Set high uncertainty for positions
        mat.set(constants::POS_X, constants::POS_X, 10.0);
        mat.set(constants::POS_Y, constants::POS_Y, 10.0);
        mat.set(constants::POS_Z, constants::POS_Z, 10.0);
        // Moderate uncertainty for velocities
        mat.set(constants::VEL_X, constants::VEL_X, 1.0);
        mat.set(constants::VEL_Y, constants::VEL_Y, 1.0);
        mat.set(constants::VEL_Z, constants::VEL_Z, 1.0);
        // High uncertainty for accelerations
        mat.set(constants::ACC_X, constants::ACC_X, 5.0);
        mat.set(constants::ACC_Y, constants::ACC_Y, 5.0);
        mat.set(constants::ACC_Z, constants::ACC_Z, 5.0);
        // Low uncertainty for orientation
        mat.set(constants::QUAT_W, constants::QUAT_W, 0.1);
        mat.set(constants::QUAT_X, constants::QUAT_X, 0.1);
        mat.set(constants::QUAT_Y, constants::QUAT_Y, 0.1);
        mat.set(constants::QUAT_Z, constants::QUAT_Z, 0.1);
        // Low uncertainty for angular velocities
        mat.set(constants::ANGVEL_X, constants::ANGVEL_X, 0.1);
        mat.set(constants::ANGVEL_Y, constants::ANGVEL_Y, 0.1);
        mat.set(constants::ANGVEL_Z, constants::ANGVEL_Z, 0.1);
        // Low uncertainty for biases
        mat.set(constants::BIAS_ACC_X, constants::BIAS_ACC_X, 0.1);
        mat.set(constants::BIAS_ACC_Y, constants::BIAS_ACC_Y, 0.1);
        mat.set(constants::BIAS_ACC_Z, constants::BIAS_ACC_Z, 0.1);
        mat.set(constants::BIAS_GYRO_X, constants::BIAS_GYRO_X, 0.1);
        mat.set(constants::BIAS_GYRO_Y, constants::BIAS_GYRO_Y, 0.1);
        mat.set(constants::BIAS_GYRO_Z, constants::BIAS_GYRO_Z, 0.1);
    });
}

/// Initializes the process noise covariance matrix with different variances.
pub fn initialize_process_noise(filter: &mut Filter, config: &RovConfig) {
    filter.direct_process_noise_mut().apply(|mat| {
        mat.clear();
        // Position process noise (e.g., due to external forces)
        mat.set(constants::POS_X, constants::POS_X, config.process_noise);
        mat.set(constants::POS_Y, constants::POS_Y, config.process_noise);
        mat.set(constants::POS_Z, constants::POS_Z, config.process_noise);
        // Velocity process noise
        mat.set(constants::VEL_X, constants::VEL_X, config.process_noise);
        mat.set(constants::VEL_Y, constants::VEL_Y, config.process_noise);
        mat.set(constants::VEL_Z, constants::VEL_Z, config.process_noise);
        // Acceleration process noise
        mat.set(constants::ACC_X, constants::ACC_X, config.process_noise);
        mat.set(constants::ACC_Y, constants::ACC_Y, config.process_noise);
        mat.set(constants::ACC_Z, constants::ACC_Z, config.process_noise);
        // Orientation process noise
        mat.set(constants::QUAT_W, constants::QUAT_W, config.process_noise);
        mat.set(constants::QUAT_X, constants::QUAT_X, config.process_noise);
        mat.set(constants::QUAT_Y, constants::QUAT_Y, config.process_noise);
        mat.set(constants::QUAT_Z, constants::QUAT_Z, config.process_noise);
        // Angular velocity process noise
        mat.set(
            constants::ANGVEL_X,
            constants::ANGVEL_X,
            config.process_noise,
        );
        mat.set(
            constants::ANGVEL_Y,
            constants::ANGVEL_Y,
            config.process_noise,
        );
        mat.set(
            constants::ANGVEL_Z,
            constants::ANGVEL_Z,
            config.process_noise,
        );
        // Bias process noise (assuming small random walk)
        mat.set(
            constants::BIAS_ACC_X,
            constants::BIAS_ACC_X,
            config.process_noise * 0.01,
        );
        mat.set(
            constants::BIAS_ACC_Y,
            constants::BIAS_ACC_Y,
            config.process_noise * 0.01,
        );
        mat.set(
            constants::BIAS_ACC_Z,
            constants::BIAS_ACC_Z,
            config.process_noise * 0.01,
        );
        mat.set(
            constants::BIAS_GYRO_X,
            constants::BIAS_GYRO_X,
            config.process_noise * 0.01,
        );
        mat.set(
            constants::BIAS_GYRO_Y,
            constants::BIAS_GYRO_Y,
            config.process_noise * 0.01,
        );
        mat.set(
            constants::BIAS_GYRO_Z,
            constants::BIAS_GYRO_Z,
            config.process_noise * 0.01,
        );
    });
}

/// Initializes the measurement noise covariance matrix with different variances.
pub fn initialize_measurement_noise(measurement: &mut Observation, config: &RovConfig) {
    measurement
        .measurement_noise_covariance_mut()
        .apply(|noise_vec| {
            noise_vec[meas_constants::DEPTH] = config.depth_noise;
            noise_vec[meas_constants::UPOS_X] = config.upos_noise;
            noise_vec[meas_constants::UPOS_Y] = config.upos_noise;
            noise_vec[meas_constants::UPOS_Z] = config.upos_noise;
            noise_vec[meas_constants::ACC_X] = config.acc_noise;
            noise_vec[meas_constants::ACC_Y] = config.acc_noise;
            noise_vec[meas_constants::ACC_Z] = config.acc_noise;
            noise_vec[meas_constants::GYRO_X] = config.gyro_noise;
            noise_vec[meas_constants::GYRO_Y] = config.gyro_noise;
            noise_vec[meas_constants::GYRO_Z] = config.gyro_noise;
            noise_vec[meas_constants::QUAT_W] = config.orientation_noise;
            noise_vec[meas_constants::QUAT_X] = config.orientation_noise;
            noise_vec[meas_constants::QUAT_Y] = config.orientation_noise;
            noise_vec[meas_constants::QUAT_Z] = config.orientation_noise;
        });
}

// Dummy sensor functions that return Option types
pub fn get_depth() -> Option<f32> {
    // Replace with actual depth sensor reading
    Some(0.0)
}

pub fn get_upos() -> Option<Vec3A> {
    // Replace with actual UGPS position
    Some(Vec3A::new(0.0, 0.0, 0.0))
}

pub fn get_accel() -> Option<Vec3A> {
    // Replace with actual accelerometer reading
    Some(Vec3A::new(0.0, 0.0, 0.0))
}

pub fn get_gyro_raw() -> Option<Vec3A> {
    // Replace with actual gyroscope reading
    Some(Vec3A::new(0.0, 0.0, 0.0))
}

pub fn get_orientation() -> Option<Quat> {
    // Replace with actual orientation sensor reading
    Some(Quat::from_xyzw(0.0, 0.0, 0.0, 1.0))
}

/// Example main function demonstrating EKF usage.
pub fn main() {
    // Example configuration
    let config = RovConfig {
        process_noise: 0.1,
        depth_noise: 0.03,
        upos_noise: 0.5,
        acc_noise: 0.3,
        gyro_noise: 0.2,
        orientation_noise: 0.1,
    };
    let mut ekf = EkfManager::new(config);
    // Simulation loop
    let mut time = 0.0;
    const STEP_DURATION: f32 = 0.1;
    for _ in 0..100 {
        // Predict step
        ekf.predict(STEP_DURATION);
        // Get measurements using dummy functions
        let depth = get_depth();
        let upos = get_upos();
        let acc = get_accel();
        let gyro_raw = get_gyro_raw();
        let orientation = get_orientation();
        // Update measurements
        ekf.update_measurements(Measurement {
            depth,
            upos,
            acc,
            gyro_raw,
            orientation,
        });
        // Print state
        print_state(&ekf.filter, time);
        time += STEP_DURATION;
    }
}

/// Prints the current state and covariance.
pub fn print_state(filter: &KalmanFilterType<{ constants::NUM_STATES }, f32>, time: f32) {
    let state = filter.state_vector();
    let cov = filter.estimate_covariance();
    let std_x = cov.get_at(constants::POS_X, constants::POS_X).sqrt();
    let std_y = cov.get_at(constants::POS_Y, constants::POS_Y).sqrt();
    let std_z = cov.get_at(constants::POS_Z, constants::POS_Z).sqrt();
    let std_vx = cov.get_at(constants::VEL_X, constants::VEL_X).sqrt();
    let std_vy = cov.get_at(constants::VEL_Y, constants::VEL_Y).sqrt();
    let std_vz = cov.get_at(constants::VEL_Z, constants::VEL_Z).sqrt();
    println!(
        "t={:.2} s,  x={:.2} ± {:.4} m\n      y={:.2} ± {:.4} m\n      z={:.2} ± {:.4} m\n     vx={:.2} ± {:.4} m/s\n     vy={:.2} ± {:.4} m/s\n     vz={:.2} ± {:.4} m/s",
        time,
        state.get_row(constants::POS_X), std_x,
        state.get_row(constants::POS_Y), std_y,
        state.get_row(constants::POS_Z), std_z,
        state.get_row(constants::VEL_X), std_vx,
        state.get_row(constants::VEL_Y), std_vy,
        state.get_row(constants::VEL_Z), std_vz
    );
}
