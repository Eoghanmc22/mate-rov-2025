use bevy::math::Vec3A;
use minikalman::prelude::*;
use minikalman::regular::builder::{KalmanFilterBuilder, KalmanFilterControlType};
use minikalman::regular::builder::{KalmanFilterObservationType, KalmanFilterType};

// Constants for state indices
// X, Y, Z are world space axes
pub mod state_constants {
    pub const NUM_STATES: usize = 12;
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
    pub const BIAS_ACC_X: usize = 9;
    pub const BIAS_ACC_Y: usize = 10;
    pub const BIAS_ACC_Z: usize = 11;
}

// Constants for measurement indices
// X, Y, Z are world space axes
pub mod meas_constants {
    pub const NUM_OBSERVATIONS: usize = 10;
    pub const DEPTH: usize = 0;
    pub const POS_X: usize = 1;
    pub const POS_Y: usize = 2;
    pub const POS_Z: usize = 3;
    pub const VEL_X: usize = 4;
    pub const VEL_Y: usize = 5;
    pub const VEL_Z: usize = 6;
    pub const ACC_X: usize = 7;
    pub const ACC_Y: usize = 8;
    pub const ACC_Z: usize = 9;
}

// Constants for control indices
pub mod ctrl_constants {
    pub const NUM_CONTROLS: usize = 0;
}

#[derive(Debug, Default)]
pub struct Measurement {
    depth: Option<f32>,
    pos: Option<Vec3A>,
    velo: Option<Vec3A>,
    accel: Option<Vec3A>,
}

/// Configuration parameters for the ROV.
#[derive(Debug)]
pub struct KalmanConfig {
    pub pos_process_noise: f32,
    pub velo_process_noise: f32,
    pub accel_process_noise: f32,
    pub accel_bias_process_noise: f32,

    pub depth_noise: f32,
    pub pos_noise: f32,
    pub velo_noise: f32,
    pub accel_noise: f32,
}

impl Default for KalmanConfig {
    fn default() -> Self {
        Self {
            depth_noise: 0.03,
            pos_noise: 0.5,
            velo_noise: 0.1,
            accel_noise: 0.3,

            pos_process_noise: 0.2,
            velo_process_noise: 0.2,
            accel_process_noise: 0.7,
            accel_bias_process_noise: 0.005,
        }
    }
}

type Filter = KalmanFilterType<{ state_constants::NUM_STATES }, f32>;
type Observation = KalmanFilterObservationType<
    { state_constants::NUM_STATES },
    { meas_constants::NUM_OBSERVATIONS },
    f32,
>;
type ControlInput =
    KalmanFilterControlType<{ state_constants::NUM_STATES }, { ctrl_constants::NUM_CONTROLS }, f32>;

/// Manager for the Extended Kalman Filter.
pub struct EkfManager {
    filter: Filter,
    measurement: Observation,
    input: ControlInput,
    config: KalmanConfig,
}

impl EkfManager {
    /// Creates a new EKF manager with the given configuration.
    pub fn new(config: KalmanConfig, initial_guess: Measurement) -> Self {
        let builder = KalmanFilterBuilder::<{ state_constants::NUM_STATES }, f32>::default();
        let mut filter = builder.build();
        let measurement = builder
            .observations()
            .build::<{ meas_constants::NUM_OBSERVATIONS }>();
        let input = builder
            .controls()
            .build::<{ ctrl_constants::NUM_CONTROLS }>();

        // Initialize state vector
        initialize_state_vector(&mut filter, &initial_guess);

        // Initialize covariance matrix with appropriate uncertainties
        initialize_estimate_covariance(&mut filter, &initial_guess);

        // Initialize process noise with different variances per state variable
        initialize_process_noise(&mut filter, &config);

        Self {
            filter,
            measurement,
            input,
            config,
        }
    }

    /// Predicts the state at the next time step.
    pub fn predict(&mut self, delta_t: f32) {
        self.compute_transition_matrix(delta_t);

        self.filter.predict_tuned(1.0);
    }

    fn compute_transition_matrix(&mut self, delta_t: f32) {
        self.filter.state_transition_mut().apply(|mat| {
            // Clear the matrix
            mat.clear();

            // Position with respect to position, velocity, and accel
            mat.set(state_constants::POS_X, state_constants::POS_X, 1.0);
            mat.set(state_constants::POS_X, state_constants::VEL_X, delta_t);
            mat.set(
                state_constants::POS_X,
                state_constants::ACC_X,
                delta_t * delta_t / 2.0,
            );
            mat.set(state_constants::POS_Y, state_constants::POS_Y, 1.0);
            mat.set(state_constants::POS_Y, state_constants::VEL_Y, delta_t);
            mat.set(
                state_constants::POS_Y,
                state_constants::ACC_Y,
                delta_t * delta_t / 2.0,
            );
            mat.set(state_constants::POS_Z, state_constants::POS_Z, 1.0);
            mat.set(state_constants::POS_Z, state_constants::VEL_Z, delta_t);
            mat.set(
                state_constants::POS_Z,
                state_constants::ACC_Z,
                delta_t * delta_t / 2.0,
            );

            // Velocity with respect to velocity and acceleration
            mat.set(state_constants::VEL_X, state_constants::VEL_X, 1.0);
            mat.set(state_constants::VEL_X, state_constants::ACC_X, delta_t);
            mat.set(state_constants::VEL_Y, state_constants::VEL_Y, 1.0);
            mat.set(state_constants::VEL_Y, state_constants::ACC_Y, delta_t);
            mat.set(state_constants::VEL_Z, state_constants::VEL_Z, 1.0);
            mat.set(state_constants::VEL_Z, state_constants::ACC_Z, delta_t);

            // Acceleration (assuming constant acceleration)
            mat.set(state_constants::ACC_X, state_constants::ACC_X, 1.0);
            mat.set(state_constants::ACC_Y, state_constants::ACC_Y, 1.0);
            mat.set(state_constants::ACC_Z, state_constants::ACC_Z, 1.0);

            // Bias terms (assuming constant biases)
            mat.set(
                state_constants::BIAS_ACC_X,
                state_constants::BIAS_ACC_X,
                1.0,
            );
            mat.set(
                state_constants::BIAS_ACC_Y,
                state_constants::BIAS_ACC_Y,
                1.0,
            );
            mat.set(
                state_constants::BIAS_ACC_Z,
                state_constants::BIAS_ACC_Z,
                1.0,
            );
        })
    }

    /// Handles partial measurements by updating only the available measurements.
    pub fn update_measurements(&mut self, observation_input: Measurement) {
        self.measurement
            .measurement_vector_mut()
            .apply(|measurement| {
                // Reset measurement vector to zero before applying new measurements
                measurement.clear();

                // Update available measurements
                if let Some(depth) = observation_input.depth {
                    measurement.set_row(meas_constants::DEPTH, depth);
                }
                if let Some(pos) = observation_input.pos {
                    measurement.set_row(meas_constants::POS_X, pos.x);
                    measurement.set_row(meas_constants::POS_Y, pos.y);
                    measurement.set_row(meas_constants::POS_Z, pos.z);
                }
                if let Some(velo) = observation_input.velo {
                    measurement.set_row(meas_constants::VEL_X, velo.x);
                    measurement.set_row(meas_constants::VEL_Y, velo.y);
                    measurement.set_row(meas_constants::VEL_Z, velo.z);
                }
                if let Some(accel) = observation_input.accel {
                    measurement.set_row(meas_constants::ACC_X, accel.x);
                    measurement.set_row(meas_constants::ACC_Y, accel.y);
                    measurement.set_row(meas_constants::ACC_Z, accel.z);
                }
            });

        // Adjust measurement noise covariance for available and unavailable measurements
        self.adjust_measurement_noise(&observation_input);

        // Adjust measurement observation matrix for available and unavailable measurements
        self.adjust_observation_matrix(&observation_input);

        // Corrects the state based on the current measurement.
        self.filter.correct(&mut self.measurement);
    }

    /// Adjusts the measurement noise covariance matrix based on available measurements.
    /// Unavailable measurements are set to a high variance to effectively ignore them.
    fn adjust_measurement_noise(&mut self, observation_input: &Measurement) {
        self.measurement
            .measurement_noise_covariance_mut()
            .apply(|noise_vec| {
                // Set high variance for all measurements initially
                for i in 0..meas_constants::NUM_OBSERVATIONS {
                    noise_vec[i] = 1e9; // Represents infinity
                }

                // Assign actual noise values to available measurements
                if observation_input.depth.is_some() {
                    noise_vec[meas_constants::DEPTH] = self.config.depth_noise;
                }
                if observation_input.pos.is_some() {
                    noise_vec[meas_constants::POS_X] = self.config.pos_noise;
                    noise_vec[meas_constants::POS_Y] = self.config.pos_noise;
                    noise_vec[meas_constants::POS_Z] = self.config.pos_noise;
                }
                if observation_input.velo.is_some() {
                    noise_vec[meas_constants::VEL_X] = self.config.velo_noise;
                    noise_vec[meas_constants::VEL_Y] = self.config.velo_noise;
                    noise_vec[meas_constants::VEL_Z] = self.config.velo_noise;
                }
                if observation_input.accel.is_some() {
                    noise_vec[meas_constants::ACC_X] = self.config.accel_noise;
                    noise_vec[meas_constants::ACC_Y] = self.config.accel_noise;
                    noise_vec[meas_constants::ACC_Z] = self.config.accel_noise;
                }
            });
    }

    /// Initializes the observation matrtix to map the state vector into observation space
    fn adjust_observation_matrix(&mut self, observation_input: &Measurement) {
        self.measurement.observation_matrix_mut().apply(|mat| {
            mat.clear();

            if observation_input.depth.is_some() {
                mat.set(meas_constants::DEPTH, state_constants::POS_Z, 1.0);
            }

            if observation_input.pos.is_some() {
                mat.set(meas_constants::POS_X, state_constants::POS_X, 1.0);
                mat.set(meas_constants::POS_Y, state_constants::POS_Y, 1.0);
                mat.set(meas_constants::POS_Z, state_constants::POS_Z, 1.0);
            }

            if observation_input.velo.is_some() {
                mat.set(meas_constants::VEL_X, state_constants::VEL_X, 1.0);
                mat.set(meas_constants::VEL_Y, state_constants::VEL_Y, 1.0);
                mat.set(meas_constants::VEL_Z, state_constants::VEL_Z, 1.0);
            }

            if observation_input.accel.is_some() {
                mat.set(meas_constants::ACC_X, state_constants::ACC_X, 1.0);
                mat.set(meas_constants::ACC_Y, state_constants::ACC_Y, 1.0);
                mat.set(meas_constants::ACC_Z, state_constants::ACC_Z, 1.0);

                mat.set(meas_constants::ACC_X, state_constants::BIAS_ACC_X, 1.0);
                mat.set(meas_constants::ACC_Y, state_constants::BIAS_ACC_Y, 1.0);
                mat.set(meas_constants::ACC_Z, state_constants::BIAS_ACC_Z, 1.0);
            }
        });
    }

    pub fn assume_state(&mut self, guess: Measurement) {
        // Initialize state vector
        initialize_state_vector(&mut self.filter, &guess);

        // Initialize covariance matrix with appropriate uncertainties
        initialize_estimate_covariance(&mut self.filter, &guess);
    }
}

/// Initializes the state vector with default values.
/// If initial measurements are available, they can be used here for better initialization.
fn initialize_state_vector(filter: &mut Filter, initial_guess: &Measurement) {
    filter.state_vector_mut().apply(|state| {
        state[state_constants::POS_X] = initial_guess.pos.map(|it| it.x).unwrap_or(0.0);
        state[state_constants::POS_Y] = initial_guess.pos.map(|it| it.y).unwrap_or(0.0);
        state[state_constants::POS_Z] = initial_guess
            .depth
            .or(initial_guess.pos.map(|it| it.z))
            .unwrap_or(0.0);

        state[state_constants::VEL_X] = initial_guess.velo.map(|it| it.x).unwrap_or(0.0);
        state[state_constants::VEL_Y] = initial_guess.velo.map(|it| it.y).unwrap_or(0.0);
        state[state_constants::VEL_Z] = initial_guess.velo.map(|it| it.z).unwrap_or(0.0);

        state[state_constants::ACC_X] = initial_guess.accel.map(|it| it.x).unwrap_or(0.0);
        state[state_constants::ACC_Y] = initial_guess.accel.map(|it| it.y).unwrap_or(0.0);
        state[state_constants::ACC_Z] = initial_guess.accel.map(|it| it.z).unwrap_or(0.0);

        state[state_constants::BIAS_ACC_X] = 0.0;
        state[state_constants::BIAS_ACC_Y] = 0.0;
        state[state_constants::BIAS_ACC_Z] = 0.0;
    });
}

/// Initializes the estimate covariance matrix with different uncertainties.
fn initialize_estimate_covariance(filter: &mut Filter, initial_guess: &Measurement) {
    filter.estimate_covariance_mut().apply(|mat| {
        mat.clear();
        if initial_guess.pos.is_some() {
            // Set moderate uncertainty for positions
            mat.set(state_constants::POS_X, state_constants::POS_X, 1.0);
            mat.set(state_constants::POS_Y, state_constants::POS_Y, 1.0);
            mat.set(state_constants::POS_Z, state_constants::POS_Z, 1.0);
        } else {
            // Set high uncertainty for positions
            mat.set(state_constants::POS_X, state_constants::POS_X, 10.0);
            mat.set(state_constants::POS_Y, state_constants::POS_Y, 10.0);
            mat.set(state_constants::POS_Z, state_constants::POS_Z, 10.0);
        }
        // Moderate uncertainty for velocities
        mat.set(state_constants::VEL_X, state_constants::VEL_X, 1.0);
        mat.set(state_constants::VEL_Y, state_constants::VEL_Y, 1.0);
        mat.set(state_constants::VEL_Z, state_constants::VEL_Z, 1.0);
        // High uncertainty for accelerations
        mat.set(state_constants::ACC_X, state_constants::ACC_X, 5.0);
        mat.set(state_constants::ACC_Y, state_constants::ACC_Y, 5.0);
        mat.set(state_constants::ACC_Z, state_constants::ACC_Z, 5.0);
        // Low uncertainty for biases
        mat.set(
            state_constants::BIAS_ACC_X,
            state_constants::BIAS_ACC_X,
            0.1,
        );
        mat.set(
            state_constants::BIAS_ACC_Y,
            state_constants::BIAS_ACC_Y,
            0.1,
        );
        mat.set(
            state_constants::BIAS_ACC_Z,
            state_constants::BIAS_ACC_Z,
            0.1,
        );
    });
}

/// Initializes the process noise covariance matrix with different variances.
fn initialize_process_noise(filter: &mut Filter, config: &KalmanConfig) {
    filter.direct_process_noise_mut().apply(|mat| {
        mat.clear();

        // Position process noise (e.g., due to external forces)
        mat.set(
            state_constants::POS_X,
            state_constants::POS_X,
            config.pos_process_noise,
        );
        mat.set(
            state_constants::POS_Y,
            state_constants::POS_Y,
            config.pos_process_noise,
        );
        mat.set(
            state_constants::POS_Z,
            state_constants::POS_Z,
            config.pos_process_noise,
        );

        // Velocity process noise
        mat.set(
            state_constants::VEL_X,
            state_constants::VEL_X,
            config.velo_process_noise,
        );
        mat.set(
            state_constants::VEL_Y,
            state_constants::VEL_Y,
            config.velo_process_noise,
        );
        mat.set(
            state_constants::VEL_Z,
            state_constants::VEL_Z,
            config.velo_process_noise,
        );

        // Acceleration process noise
        mat.set(
            state_constants::ACC_X,
            state_constants::ACC_X,
            config.accel_process_noise,
        );
        mat.set(
            state_constants::ACC_Y,
            state_constants::ACC_Y,
            config.accel_process_noise,
        );
        mat.set(
            state_constants::ACC_Z,
            state_constants::ACC_Z,
            config.accel_process_noise,
        );

        // Bias process noise (assuming small random walk)
        mat.set(
            state_constants::BIAS_ACC_X,
            state_constants::BIAS_ACC_X,
            config.accel_bias_process_noise,
        );
        mat.set(
            state_constants::BIAS_ACC_Y,
            state_constants::BIAS_ACC_Y,
            config.accel_bias_process_noise,
        );
        mat.set(
            state_constants::BIAS_ACC_Z,
            state_constants::BIAS_ACC_Z,
            config.accel_bias_process_noise,
        );
    });
}

#[cfg(test)]
mod tests {
    use core::f32;

    use bevy::math::Vec3A;
    use minikalman::matrix::{RowMajorSequentialData, RowVector};
    use rand::random;
    use rand_distr::{Distribution, Normal};

    use crate::{
        kalman_filter::simple_filter::{EkfManager, KalmanConfig, Measurement},
        trajectory,
    };

    use super::{state_constants, Filter};

    #[test]
    fn simple_kalman_filter() {
        // Example configuration
        let config = KalmanConfig::default();
        let mut ekf = EkfManager::new(config, Default::default());
        // Simulation loop
        let mut time = 0.0;
        const STEP_DURATION: f32 = 0.1;

        let trajectory = std::iter::from_coroutine(
            #[coroutine]
            || {
                let radius = 3.0;
                let freq = f32::consts::FRAC_PI_2;
                let vertical_speed = 1.5;
                let vertical_accel = 0.5;

                let mut rng = rand::rng();
                let pos_dist = Normal::new(0.0, 0.7).unwrap();
                let vel_dist = Normal::new(0.0, 0.2).unwrap();
                let acc_dist = Normal::new(0.0, 0.4).unwrap();
                let depth_dist = Normal::new(0.0, 0.02).unwrap();

                let starting_x = random::<f32>() * 20.0;
                let starting_y = random::<f32>() * 20.0;
                let starting_z = random::<f32>() * 20.0;

                for time_step in 0..3 {
                    let time = STEP_DURATION * time_step as f32;

                    let x = (time * freq).cos() * radius + starting_x;
                    let y = (time * freq).sin() * radius + starting_y;
                    let z = 0.5 * time * time * vertical_accel + time * vertical_speed + starting_z;

                    let vx = -(time * freq).sin() * radius * freq;
                    let vy = (time * freq).cos() * radius * freq;
                    let vz = time * vertical_accel + vertical_speed;

                    let ax = -(time * freq).cos() * radius * freq * freq;
                    let ay = -(time * freq).sin() * radius * freq * freq;
                    let az = vertical_accel;

                    println!("x: {x:.2}, y: {y:.2}, z: {z:.2}, vx: {vx:.2}, vy: {vy:.2}, vz: {vz:.2}, ax: {ax:.2}, ay: {ay:.2}, az: {az:.2}");
                    yield Measurement {
                        depth: Some(z + depth_dist.sample(&mut rng)),
                        // depth: None,
                        pos: Some(Vec3A::new(
                            x + pos_dist.sample(&mut rng),
                            y + pos_dist.sample(&mut rng),
                            z + pos_dist.sample(&mut rng),
                        )),
                        // pos: None,
                        velo: Some(Vec3A::new(
                            vx + vel_dist.sample(&mut rng),
                            vy + vel_dist.sample(&mut rng),
                            vz + vel_dist.sample(&mut rng),
                        )),
                        // velo: None,
                        accel: Some(Vec3A::new(
                            ax + acc_dist.sample(&mut rng),
                            ay + acc_dist.sample(&mut rng),
                            az + acc_dist.sample(&mut rng),
                        )),
                        // accel: None,
                    };
                }
            },
        );

        for measurement in trajectory {
            print_state(&ekf.filter, time);

            // Predict step
            ekf.predict(STEP_DURATION);
            print_state(&ekf.filter, time);

            // Update measurements
            ekf.update_measurements(measurement);
            // Print state
            print_state(&ekf.filter, time);
            time += STEP_DURATION;
        }

        panic!();
    }

    /// Prints the current state and covariance.
    pub fn print_state(filter: &Filter, time: f32) {
        let state = filter.state_vector();
        let cov = filter.estimate_covariance();
        let std_x = cov
            .get_at(state_constants::POS_X, state_constants::POS_X)
            .sqrt();
        let std_y = cov
            .get_at(state_constants::POS_Y, state_constants::POS_Y)
            .sqrt();
        let std_z = cov
            .get_at(state_constants::POS_Z, state_constants::POS_Z)
            .sqrt();
        let std_vx = cov
            .get_at(state_constants::VEL_X, state_constants::VEL_X)
            .sqrt();
        let std_vy = cov
            .get_at(state_constants::VEL_Y, state_constants::VEL_Y)
            .sqrt();
        let std_vz = cov
            .get_at(state_constants::VEL_Z, state_constants::VEL_Z)
            .sqrt();
        let std_ax = cov
            .get_at(state_constants::ACC_X, state_constants::ACC_X)
            .sqrt();
        let std_ay = cov
            .get_at(state_constants::ACC_Y, state_constants::ACC_Y)
            .sqrt();
        let std_az = cov
            .get_at(state_constants::ACC_Z, state_constants::ACC_Z)
            .sqrt();
        println!(
        "t={:.2} s\n      x={:.2} ± {:.4} m\n      y={:.2} ± {:.4} m\n      z={:.2} ± {:.4} m\n     vx={:.2} ± {:.4} m/s\n     vy={:.2} ± {:.4} m/s\n     vz={:.2} ± {:.4} m/s\n     ax={:.2} ± {:.4} m/s^2\n     ay={:.2} ± {:.4} m/s^2\n     az={:.2} ± {:.4} m/s^2",
        time,
        state.get_row(state_constants::POS_X), std_x,
        state.get_row(state_constants::POS_Y), std_y,
        state.get_row(state_constants::POS_Z), std_z,
        state.get_row(state_constants::VEL_X), std_vx,
        state.get_row(state_constants::VEL_Y), std_vy,
        state.get_row(state_constants::VEL_Z), std_vz,
        state.get_row(state_constants::ACC_X), std_ax,
        state.get_row(state_constants::ACC_Y), std_ay,
        state.get_row(state_constants::ACC_Z), std_az
    );
    }
}
