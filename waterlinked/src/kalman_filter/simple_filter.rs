// TODO: Consider taking acceleration out of the kalman filter again
use adskalman::{KalmanFilterNoControl, ObservationModel, TransitionModelLinearNoControl};
use bevy::math::Vec3A;
use nalgebra::{Const, Matrix, OMatrix, OVector, Owned};

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
            depth_noise: 0.02,
            pos_noise: 0.7,
            velo_noise: 0.2,
            accel_noise: 0.4,

            pos_process_noise: 0.2,
            velo_process_noise: 0.2,
            accel_process_noise: 2.0,
            accel_bias_process_noise: 0.001,
        }
    }
}

type R = f32;
type SS = Const<{ state_constants::NUM_STATES }>;
type OS = Const<{ meas_constants::NUM_OBSERVATIONS }>;

struct ROVTransitionModel {
    transition: Matrix<R, SS, SS, Owned<R, SS, SS>>,
    transition_transpose: Matrix<R, SS, SS, Owned<R, SS, SS>>,
    process_covariance: Matrix<R, SS, SS, Owned<R, SS, SS>>,
}

impl ROVTransitionModel {
    pub fn new(config: &KalmanConfig, delta_t: R) -> Self {
        let mut transition = OMatrix::<R, SS, SS>::zeros();

        // Position with respect to position, velocity, and accel
        transition[(state_constants::POS_X, state_constants::POS_X)] = 1.0;
        transition[(state_constants::POS_X, state_constants::VEL_X)] = delta_t;
        transition[(state_constants::POS_X, state_constants::ACC_X)] = delta_t * delta_t / 2.0;
        transition[(state_constants::POS_Y, state_constants::POS_Y)] = 1.0;
        transition[(state_constants::POS_Y, state_constants::VEL_Y)] = delta_t;
        transition[(state_constants::POS_Y, state_constants::ACC_Y)] = delta_t * delta_t / 2.0;
        transition[(state_constants::POS_Z, state_constants::POS_Z)] = 1.0;
        transition[(state_constants::POS_Z, state_constants::VEL_Z)] = delta_t;
        transition[(state_constants::POS_Z, state_constants::ACC_Z)] = delta_t * delta_t / 2.0;

        // Velocity with respect to velocity and acceleration
        transition[(state_constants::VEL_X, state_constants::VEL_X)] = 1.0;
        transition[(state_constants::VEL_X, state_constants::ACC_X)] = delta_t;
        transition[(state_constants::VEL_Y, state_constants::VEL_Y)] = 1.0;
        transition[(state_constants::VEL_Y, state_constants::ACC_Y)] = delta_t;
        transition[(state_constants::VEL_Z, state_constants::VEL_Z)] = 1.0;
        transition[(state_constants::VEL_Z, state_constants::ACC_Z)] = delta_t;

        // Acceleration (assuming constant acceleration)
        transition[(state_constants::ACC_X, state_constants::ACC_X)] = 1.0;
        transition[(state_constants::ACC_Y, state_constants::ACC_Y)] = 1.0;
        transition[(state_constants::ACC_Z, state_constants::ACC_Z)] = 1.0;

        // Bias terms (assuming constant biases)
        transition[(state_constants::BIAS_ACC_X, state_constants::BIAS_ACC_X)] = 1.0;
        transition[(state_constants::BIAS_ACC_Y, state_constants::BIAS_ACC_Y)] = 1.0;
        transition[(state_constants::BIAS_ACC_Z, state_constants::BIAS_ACC_Z)] = 1.0;

        let transition_transpose = transition.transpose();

        let mut process_covariance = OMatrix::<R, SS, SS>::zeros();
        // Position process noise (e.g., due to external forces)
        process_covariance[(state_constants::POS_X, state_constants::POS_X)] =
            config.pos_process_noise;
        process_covariance[(state_constants::POS_Y, state_constants::POS_Y)] =
            config.pos_process_noise;
        process_covariance[(state_constants::POS_Z, state_constants::POS_Z)] =
            config.pos_process_noise;

        // Velocity process noise
        process_covariance[(state_constants::VEL_X, state_constants::VEL_X)] =
            config.velo_process_noise;
        process_covariance[(state_constants::VEL_Y, state_constants::VEL_Y)] =
            config.velo_process_noise;
        process_covariance[(state_constants::VEL_Z, state_constants::VEL_Z)] =
            config.velo_process_noise;

        // Acceleration process noise
        process_covariance[(state_constants::ACC_X, state_constants::ACC_X)] =
            config.accel_process_noise;
        process_covariance[(state_constants::ACC_Y, state_constants::ACC_Y)] =
            config.accel_process_noise;
        process_covariance[(state_constants::ACC_Z, state_constants::ACC_Z)] =
            config.accel_process_noise;

        // Bias process noise (assuming small random walk)
        process_covariance[(state_constants::BIAS_ACC_X, state_constants::BIAS_ACC_X)] =
            config.accel_bias_process_noise;
        process_covariance[(state_constants::BIAS_ACC_Y, state_constants::BIAS_ACC_Y)] =
            config.accel_bias_process_noise;
        process_covariance[(state_constants::BIAS_ACC_Z, state_constants::BIAS_ACC_Z)] =
            config.accel_bias_process_noise;

        Self {
            transition,
            transition_transpose,
            process_covariance,
        }
    }
}

impl TransitionModelLinearNoControl<R, SS> for ROVTransitionModel {
    fn F(&self) -> &Matrix<R, SS, SS, Owned<R, SS, SS>> {
        &self.transition
    }

    fn FT(&self) -> &Matrix<R, SS, SS, Owned<R, SS, SS>> {
        &self.transition_transpose
    }

    fn Q(&self) -> &Matrix<R, SS, SS, Owned<R, SS, SS>> {
        &self.process_covariance
    }
}

struct ROVObservationModel {
    observation: Matrix<R, OS, SS, Owned<R, OS, SS>>,
    observation_transpose: Matrix<R, SS, OS, Owned<R, SS, OS>>,
    observation_covariance: Matrix<R, OS, OS, Owned<R, OS, OS>>,
}

impl ROVObservationModel {
    pub fn new(config: &KalmanConfig) -> Self {
        let mut observation = OMatrix::<R, OS, SS>::zeros();

        observation[(meas_constants::DEPTH, state_constants::POS_Z)] = 1.0;

        observation[(meas_constants::POS_X, state_constants::POS_X)] = 1.0;
        observation[(meas_constants::POS_Y, state_constants::POS_Y)] = 1.0;
        observation[(meas_constants::POS_Z, state_constants::POS_Z)] = 1.0;

        observation[(meas_constants::VEL_X, state_constants::VEL_X)] = 1.0;
        observation[(meas_constants::VEL_Y, state_constants::VEL_Y)] = 1.0;
        observation[(meas_constants::VEL_Z, state_constants::VEL_Z)] = 1.0;

        observation[(meas_constants::ACC_X, state_constants::ACC_X)] = 1.0;
        observation[(meas_constants::ACC_Y, state_constants::ACC_Y)] = 1.0;
        observation[(meas_constants::ACC_Z, state_constants::ACC_Z)] = 1.0;

        observation[(meas_constants::ACC_X, state_constants::BIAS_ACC_X)] = 1.0;
        observation[(meas_constants::ACC_Y, state_constants::BIAS_ACC_Y)] = 1.0;
        observation[(meas_constants::ACC_Z, state_constants::BIAS_ACC_Z)] = 1.0;

        let observation_transpose = observation.transpose();

        let mut observation_noise = OVector::<R, OS>::zeros();
        observation_noise[meas_constants::DEPTH] = config.depth_noise;
        observation_noise[meas_constants::POS_X] = config.pos_noise;
        observation_noise[meas_constants::POS_Y] = config.pos_noise;
        observation_noise[meas_constants::POS_Z] = config.pos_noise;
        observation_noise[meas_constants::VEL_X] = config.velo_noise;
        observation_noise[meas_constants::VEL_Y] = config.velo_noise;
        observation_noise[meas_constants::VEL_Z] = config.velo_noise;
        observation_noise[meas_constants::ACC_X] = config.accel_noise;
        observation_noise[meas_constants::ACC_Y] = config.accel_noise;
        observation_noise[meas_constants::ACC_Z] = config.accel_noise;

        let observation_covariance = OMatrix::<R, OS, OS>::from_diagonal(&observation_noise);

        Self {
            observation,
            observation_transpose,
            observation_covariance,
        }
    }
}

impl ObservationModel<R, SS, OS> for ROVObservationModel {
    fn H(&self) -> &Matrix<R, OS, SS, Owned<R, OS, SS>> {
        &self.observation
    }

    fn HT(&self) -> &Matrix<R, SS, OS, Owned<R, SS, OS>> {
        &self.observation_transpose
    }

    fn R(&self) -> &Matrix<R, OS, OS, Owned<R, OS, OS>> {
        &self.observation_covariance
    }
}

#[cfg(test)]
mod tests {
    use core::f32;

    use adskalman::{KalmanFilterNoControl, StateAndCovariance};
    use bevy::math::Vec3A;
    use nalgebra::{OMatrix, OVector};
    use rand::random;
    use rand_distr::{Distribution, Normal};

    use crate::{
        kalman_filter::simple_filter::{
            KalmanConfig, Measurement, ROVObservationModel, ROVTransitionModel, OS,
        },
        trajectory,
    };

    use super::{state_constants, R, SS};

    #[test]
    fn simple_kalman_filter() {
        const STEP_DURATION: f32 = 0.1;

        let config = KalmanConfig::default();
        let transition_model = ROVTransitionModel::new(&config, STEP_DURATION);
        let observation_model = ROVObservationModel::new(&config);
        let filter = KalmanFilterNoControl::new(&transition_model, &observation_model);

        // Simulation loop
        let mut time = 0.0;

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

                let acc_bias_x = random::<f32>() * 0.3;
                let acc_bias_y = random::<f32>() * 0.3 + 9.81;
                let acc_bias_z = random::<f32>() * 0.3;

                for time_step in 0..1000 {
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

                    println!("x: {x:.2}, y: {y:.2}, z: {z:.2}, vx: {vx:.2}, vy: {vy:.2}, vz: {vz:.2}, ax: {ax:.2}, ay: {ay:.2}, az: {az:.2}, abx: {acc_bias_x:.2}, aby: {acc_bias_y:.2}, abz: {acc_bias_z:.2}");
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
                            ax + acc_dist.sample(&mut rng) + acc_bias_x,
                            ay + acc_dist.sample(&mut rng) + acc_bias_y,
                            az + acc_dist.sample(&mut rng) + acc_bias_z,
                        )),
                        // accel: None,
                    };
                }
            },
        );

        let mut cov_diag = OVector::<R, SS>::zeros();
        // Set high uncertainty for positions
        cov_diag[state_constants::POS_X] = 10.0;
        cov_diag[state_constants::POS_Y] = 10.0;
        cov_diag[state_constants::POS_Z] = 10.0;
        // Moderate uncertainty for velocities
        cov_diag[state_constants::VEL_X] = 1.0;
        cov_diag[state_constants::VEL_Y] = 1.0;
        cov_diag[state_constants::VEL_Z] = 1.0;
        // High uncertainty for accelerations
        cov_diag[state_constants::ACC_X] = 5.0;
        cov_diag[state_constants::ACC_Y] = 5.0;
        cov_diag[state_constants::ACC_Z] = 5.0;
        // Low uncertainty for biases
        cov_diag[state_constants::BIAS_ACC_X] = 0.1;
        cov_diag[state_constants::BIAS_ACC_Y] = 0.1;
        cov_diag[state_constants::BIAS_ACC_Z] = 0.1;

        let mut state = StateAndCovariance::new(
            OVector::<R, SS>::zeros(),
            OMatrix::<R, SS, SS>::from_diagonal(&cov_diag),
        );
        for measurement in trajectory {
            let observation = OVector::<R, OS>::from_vec(vec![
                measurement.depth.unwrap(),
                measurement.pos.unwrap().x,
                measurement.pos.unwrap().y,
                measurement.pos.unwrap().z,
                measurement.velo.unwrap().x,
                measurement.velo.unwrap().y,
                measurement.velo.unwrap().z,
                measurement.accel.unwrap().x,
                measurement.accel.unwrap().y,
                measurement.accel.unwrap().z,
            ]);

            // Predict step
            state = filter.step(&state, &observation).unwrap();
            print_state(&state, time);

            time += STEP_DURATION;
        }

        panic!();
    }

    /// Prints the current state and covariance.
    pub fn print_state(state_cov: &StateAndCovariance<R, SS>, time: f32) {
        let state = state_cov.state();
        let cov = state_cov.covariance();
        let std_x = cov[(state_constants::POS_X, state_constants::POS_X)].sqrt();
        let std_y = cov[(state_constants::POS_Y, state_constants::POS_Y)].sqrt();
        let std_z = cov[(state_constants::POS_Z, state_constants::POS_Z)].sqrt();
        let std_vx = cov[(state_constants::VEL_X, state_constants::VEL_X)].sqrt();
        let std_vy = cov[(state_constants::VEL_Y, state_constants::VEL_Y)].sqrt();
        let std_vz = cov[(state_constants::VEL_Z, state_constants::VEL_Z)].sqrt();
        let std_ax = cov[(state_constants::ACC_X, state_constants::ACC_X)].sqrt();
        let std_ay = cov[(state_constants::ACC_Y, state_constants::ACC_Y)].sqrt();
        let std_az = cov[(state_constants::ACC_Z, state_constants::ACC_Z)].sqrt();
        let std_abx = cov[(state_constants::BIAS_ACC_X, state_constants::BIAS_ACC_X)].sqrt();
        let std_aby = cov[(state_constants::BIAS_ACC_Y, state_constants::BIAS_ACC_Y)].sqrt();
        let std_abz = cov[(state_constants::BIAS_ACC_Z, state_constants::BIAS_ACC_Z)].sqrt();
        println!(
            "t={:.2} s\n      x={:.2} ± {:.4} m\n      y={:.2} ± {:.4} m\n      z={:.2} ± {:.4} m\n     vx={:.2} ± {:.4} m/s\n     vy={:.2} ± {:.4} m/s\n     vz={:.2} ± {:.4} m/s\n     ax={:.2} ± {:.4} m/s^2\n     ay={:.2} ± {:.4} m/s^2\n     az={:.2} ± {:.4} m/s^2\n     abx={:.2} ± {:.4} m/s^2\n     aby={:.2} ± {:.4} m/s^2\n     abz={:.2} ± {:.4} m/s^2",
            time,
            state[state_constants::POS_X], std_x,
            state[state_constants::POS_Y], std_y,
            state[state_constants::POS_Z], std_z,
            state[state_constants::VEL_X], std_vx,
            state[state_constants::VEL_Y], std_vy,
            state[state_constants::VEL_Z], std_vz,
            state[state_constants::ACC_X], std_ax,
            state[state_constants::ACC_Y], std_ay,
            state[state_constants::ACC_Z], std_az,
            state[state_constants::BIAS_ACC_X], std_abx,
            state[state_constants::BIAS_ACC_Y], std_aby,
            state[state_constants::BIAS_ACC_Z], std_abz
        );
    }
}
