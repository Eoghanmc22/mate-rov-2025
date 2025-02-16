use core::f32;

use bevy::math::Vec3A;
use kfilter::{
    measurement::LinearMeasurement,
    system::{LinearSystem, System},
    Kalman, KalmanFilter,
};
use nalgebra::{SMatrix, SVector};

// Constants for state indices
// X, Y, Z are world space axes
pub mod state_constants {
    pub const NUM_STATES: usize = 9;
    pub const POS_X: usize = 0;
    pub const POS_Y: usize = 1;
    pub const POS_Z: usize = 2;
    pub const VEL_X: usize = 3;
    pub const VEL_Y: usize = 4;
    pub const VEL_Z: usize = 5;
    pub const BIAS_ACC_X: usize = 6;
    pub const BIAS_ACC_Y: usize = 7;
    pub const BIAS_ACC_Z: usize = 8;
}

// Constants for measurement indices
// X, Y, Z are world space axes
pub mod meas_constants {
    // pub const NUM_OBSERVATIONS: usize = 4;
    pub const NUM_OBSERVATIONS: usize = 3;
    // pub const DEPTH: usize = 0;
    // pub const POS_X: usize = 1;
    // pub const POS_Y: usize = 2;
    // pub const POS_Z: usize = 3;
    pub const POS_X: usize = 0;
    pub const POS_Y: usize = 1;
    pub const POS_Z: usize = 2;
    // pub const VEL_X: usize = 4;
    // pub const VEL_Y: usize = 5;
    // pub const VEL_Z: usize = 6;
    // pub const ACC_X: usize = 7;
    // pub const ACC_Y: usize = 8;
    // pub const ACC_Z: usize = 9;
}

// Constants for control indices
pub mod ctrl_constants {
    pub const NUM_CONTROLS: usize = 3;
    pub const ACC_X: usize = 0;
    pub const ACC_Y: usize = 1;
    pub const ACC_Z: usize = 2;
}

#[derive(Debug, Default)]
pub struct Measurement {
    depth: Option<f32>,
    pos: Option<Vec3A>,
    // velo: Option<Vec3A>,
    accel: Option<Vec3A>,
}

/// Configuration parameters for the ROV.
#[derive(Debug)]
pub struct KalmanConfig {
    pub pos_process_noise: f32,
    pub velo_process_noise: f32,
    // pub accel_process_noise: f32,
    pub accel_bias_process_noise: f32,

    pub depth_noise: f32,
    pub pos_noise: f32,
    // pub velo_noise: f32,
    pub accel_noise: f32,
}

impl Default for KalmanConfig {
    fn default() -> Self {
        Self {
            depth_noise: 0.02,
            pos_noise: 0.7,
            // velo_noise: 0.2,
            accel_noise: 0.4,

            pos_process_noise: 0.2,
            velo_process_noise: 0.2,
            accel_bias_process_noise: 0.001,
        }
    }
}

pub type Filter =
    Kalman<f32, { state_constants::NUM_STATES }, { ctrl_constants::NUM_CONTROLS }, Dynamics>;
pub type Dynamics =
    LinearSystem<f32, { state_constants::NUM_STATES }, { ctrl_constants::NUM_CONTROLS }>;
pub type Observation =
    LinearMeasurement<f32, { state_constants::NUM_STATES }, { meas_constants::NUM_OBSERVATIONS }>;

pub fn make_filter(config: &KalmanConfig, delta_t: f32, initial_position: Option<Vec3A>) -> Filter {
    let mut initial_state = SMatrix::zeros();

    if let Some(initial_position) = initial_position {
        initial_state[state_constants::POS_X] = initial_position.x;
        initial_state[state_constants::POS_Y] = initial_position.y;
        initial_state[state_constants::POS_Z] = initial_position.z;
    }

    let system = make_system(config, delta_t, initial_state);

    let mut init_cov_diag = SVector::<f32, { state_constants::NUM_STATES }>::zeros();
    if initial_position.is_some() {
        // Set moderate uncertainty for positions when we have a good initial guess
        init_cov_diag[state_constants::POS_X] = 1.0;
        init_cov_diag[state_constants::POS_Y] = 1.0;
        init_cov_diag[state_constants::POS_Z] = 1.0;
    } else {
        // Set high uncertainty for positions when its unknown
        init_cov_diag[state_constants::POS_X] = 10.0;
        init_cov_diag[state_constants::POS_Y] = 10.0;
        init_cov_diag[state_constants::POS_Z] = 10.0;
    }
    // Moderate uncertainty for velocities
    init_cov_diag[state_constants::VEL_X] = 1.0;
    init_cov_diag[state_constants::VEL_Y] = 1.0;
    init_cov_diag[state_constants::VEL_Z] = 1.0;
    // High uncertainty for accelerations
    // cov_diag[state_constants::ACC_X] = 5.0;
    // cov_diag[state_constants::ACC_Y] = 5.0;
    // cov_diag[state_constants::ACC_Z] = 5.0;
    // Low uncertainty for biases
    init_cov_diag[state_constants::BIAS_ACC_X] = 0.1;
    init_cov_diag[state_constants::BIAS_ACC_Y] = 0.1;
    init_cov_diag[state_constants::BIAS_ACC_Z] = 0.1;

    let init_cov = SMatrix::<f32, { state_constants::NUM_STATES }, { state_constants::NUM_STATES }>::from_diagonal(
        &init_cov_diag,
    );

    Kalman::new_custom(system, init_cov)
}

pub fn make_system(
    config: &KalmanConfig,
    delta_t: f32,
    state: SVector<f32, { state_constants::NUM_STATES }>,
) -> Dynamics {
    let mut transition =
        SMatrix::<f32, { state_constants::NUM_STATES }, { state_constants::NUM_STATES }>::zeros();

    // Position with respect to position, velocity, and accel
    transition[(state_constants::POS_X, state_constants::POS_X)] = 1.0;
    transition[(state_constants::POS_X, state_constants::VEL_X)] = delta_t;
    transition[(state_constants::POS_X, state_constants::BIAS_ACC_X)] = -delta_t * delta_t / 2.0;

    transition[(state_constants::POS_Y, state_constants::POS_Y)] = 1.0;
    transition[(state_constants::POS_Y, state_constants::VEL_Y)] = delta_t;
    transition[(state_constants::POS_Y, state_constants::BIAS_ACC_Y)] = -delta_t * delta_t / 2.0;

    transition[(state_constants::POS_Z, state_constants::POS_Z)] = 1.0;
    transition[(state_constants::POS_Z, state_constants::VEL_Z)] = delta_t;
    transition[(state_constants::POS_Z, state_constants::BIAS_ACC_Z)] = -delta_t * delta_t / 2.0;

    // Velocity with respect to velocity and acceleration
    transition[(state_constants::VEL_X, state_constants::VEL_X)] = 1.0;
    transition[(state_constants::VEL_X, state_constants::BIAS_ACC_X)] = -delta_t;

    transition[(state_constants::VEL_Y, state_constants::VEL_Y)] = 1.0;
    transition[(state_constants::VEL_Y, state_constants::BIAS_ACC_Y)] = -delta_t;

    transition[(state_constants::VEL_Z, state_constants::VEL_Z)] = 1.0;
    transition[(state_constants::VEL_Z, state_constants::BIAS_ACC_Z)] = -delta_t;

    // Bias terms (assuming constant biases)
    transition[(state_constants::BIAS_ACC_X, state_constants::BIAS_ACC_X)] = 1.0;
    transition[(state_constants::BIAS_ACC_Y, state_constants::BIAS_ACC_Y)] = 1.0;
    transition[(state_constants::BIAS_ACC_Z, state_constants::BIAS_ACC_Z)] = 1.0;

    let mut process_covariance =
        SMatrix::<f32, { state_constants::NUM_STATES }, { state_constants::NUM_STATES }>::zeros();

    // Position process noise scaling (assuming constant acceleration noise)
    let q_pos = config.pos_process_noise;
    process_covariance[(state_constants::POS_X, state_constants::POS_X)] =
        q_pos * delta_t.powi(3) / 3.0;
    process_covariance[(state_constants::POS_X, state_constants::VEL_X)] =
        q_pos * delta_t.powi(2) / 2.0;
    process_covariance[(state_constants::VEL_X, state_constants::POS_X)] =
        q_pos * delta_t.powi(2) / 2.0;
    process_covariance[(state_constants::VEL_X, state_constants::VEL_X)] = q_pos * delta_t;

    let q_pos_y = config.pos_process_noise;
    process_covariance[(state_constants::POS_Y, state_constants::POS_Y)] =
        q_pos_y * delta_t.powi(3) / 3.0;
    process_covariance[(state_constants::POS_Y, state_constants::VEL_Y)] =
        q_pos_y * delta_t.powi(2) / 2.0;
    process_covariance[(state_constants::VEL_Y, state_constants::POS_Y)] =
        q_pos_y * delta_t.powi(2) / 2.0;
    process_covariance[(state_constants::VEL_Y, state_constants::VEL_Y)] = q_pos_y * delta_t;

    let q_pos_z = config.pos_process_noise;
    process_covariance[(state_constants::POS_Z, state_constants::POS_Z)] =
        q_pos_z * delta_t.powi(3) / 3.0;
    process_covariance[(state_constants::POS_Z, state_constants::VEL_Z)] =
        q_pos_z * delta_t.powi(2) / 2.0;
    process_covariance[(state_constants::VEL_Z, state_constants::POS_Z)] =
        q_pos_z * delta_t.powi(2) / 2.0;
    process_covariance[(state_constants::VEL_Z, state_constants::VEL_Z)] = q_pos_z * delta_t;

    // Bias process noise (assuming random walk, scales linearly with delta_t)
    let q_bias = config.accel_bias_process_noise;
    process_covariance[(state_constants::BIAS_ACC_X, state_constants::BIAS_ACC_X)] =
        q_bias * delta_t;
    process_covariance[(state_constants::BIAS_ACC_Y, state_constants::BIAS_ACC_Y)] =
        q_bias * delta_t;
    process_covariance[(state_constants::BIAS_ACC_Z, state_constants::BIAS_ACC_Z)] =
        q_bias * delta_t;

    // TODO: may need to introduce covatance between the biases and position and velo

    let mut control_mat =
        SMatrix::<f32, { state_constants::NUM_STATES }, { ctrl_constants::NUM_CONTROLS }>::zeros();
    control_mat[(state_constants::POS_X, ctrl_constants::ACC_X)] = delta_t * delta_t / 2.0;
    control_mat[(state_constants::POS_Y, ctrl_constants::ACC_Y)] = delta_t * delta_t / 2.0;
    control_mat[(state_constants::POS_Z, ctrl_constants::ACC_Z)] = delta_t * delta_t / 2.0;
    control_mat[(state_constants::VEL_X, ctrl_constants::ACC_X)] = delta_t;
    control_mat[(state_constants::VEL_Y, ctrl_constants::ACC_Y)] = delta_t;
    control_mat[(state_constants::VEL_Z, ctrl_constants::ACC_Z)] = delta_t;

    LinearSystem::new(transition, process_covariance, control_mat, state)
}

pub fn make_measurement(config: &KalmanConfig, measurement: &Measurement) -> Observation {
    let measurement = SVector::<f32, { meas_constants::NUM_OBSERVATIONS }>::from_vec(vec![
        // measurement.depth.unwrap(),
        measurement.pos.unwrap().x,
        measurement.pos.unwrap().y,
        measurement.pos.unwrap().z,
        // measurement.velo.unwrap().x,
        // measurement.velo.unwrap().y,
        // measurement.velo.unwrap().z,
        // measurement.accel.unwrap().x,
        // measurement.accel.unwrap().y,
        // measurement.accel.unwrap().z,
    ]);
    let mut observation = SMatrix::<
        f32,
        { meas_constants::NUM_OBSERVATIONS },
        { state_constants::NUM_STATES },
    >::zeros();

    // observation[(meas_constants::DEPTH, state_constants::POS_Z)] = 1.0;

    observation[(meas_constants::POS_X, state_constants::POS_X)] = 1.0;
    observation[(meas_constants::POS_Y, state_constants::POS_Y)] = 1.0;
    observation[(meas_constants::POS_Z, state_constants::POS_Z)] = 1.0;

    let mut observation_noise = SMatrix::<
        f32,
        { meas_constants::NUM_OBSERVATIONS },
        { meas_constants::NUM_OBSERVATIONS },
    >::zeros();
    // observation_noise[meas_constants::DEPTH] = config.depth_noise;
    observation_noise[meas_constants::POS_X] = config.pos_noise;
    observation_noise[meas_constants::POS_Y] = config.pos_noise;
    observation_noise[meas_constants::POS_Z] = config.pos_noise;
    // observation_noise[meas_constants::VEL_X] = config.velo_noise;
    // observation_noise[meas_constants::VEL_Y] = config.velo_noise;
    // observation_noise[meas_constants::VEL_Z] = config.velo_noise;
    // observation_noise[meas_constants::ACC_X] = config.accel_noise;
    // observation_noise[meas_constants::ACC_Y] = config.accel_noise;
    // observation_noise[meas_constants::ACC_Z] = config.accel_noise;

    Observation::new(observation, observation_noise, measurement)
}

#[cfg(test)]
mod tests {
    use core::f32;

    use bevy::math::Vec3A;
    use kfilter::{Kalman, KalmanFilter, KalmanPredictInput, KalmanUpdate};
    use nalgebra::{OMatrix, OVector, SVector};
    use rand::random;
    use rand_distr::{Distribution, Normal};

    use crate::kalman_filter::kfilter_control::{
        ctrl_constants, make_filter, make_measurement, make_system, meas_constants,
        state_constants, KalmanConfig, Measurement,
    };

    use super::Filter;

    #[test]
    fn simple_kalman_filter() {
        const STEP_DURATION: f32 = 0.1;

        let config = KalmanConfig::default();
        let mut filter = make_filter(&config, STEP_DURATION, None);

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
                let acc_bias_y = random::<f32>() * 0.3/* + 9.81*/;
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
                    yield (
                        Measurement {
                            depth: Some(z + depth_dist.sample(&mut rng)),
                            // depth: None,
                            pos: Some(Vec3A::new(
                                x + pos_dist.sample(&mut rng),
                                y + pos_dist.sample(&mut rng),
                                z + pos_dist.sample(&mut rng),
                            )),
                            // pos: None,
                            // velo: Some(Vec3A::new(
                            //     vx + vel_dist.sample(&mut rng),
                            //     vy + vel_dist.sample(&mut rng),
                            //     vz + vel_dist.sample(&mut rng),
                            // )),
                            // velo: None,
                            accel: Some(Vec3A::new(
                                ax + acc_dist.sample(&mut rng) + acc_bias_x,
                                ay + acc_dist.sample(&mut rng) + acc_bias_y,
                                az + acc_dist.sample(&mut rng) + acc_bias_z,
                            )),
                            // accel: None,
                        },
                        Vec3A::new(x, y, z),
                    );
                }
            },
        );

        let mut norm_acc = 0.0;
        for (measurement, ground_truth) in trajectory {
            let meas = make_measurement(&config, &measurement);

            // // Predict step
            // state = filter.step(&state, &observation).unwrap();
            // print_state(&state, time);

            let ctrl_input = SVector::<f32, { ctrl_constants::NUM_CONTROLS }>::from_vec(vec![
                measurement.accel.unwrap().x,
                measurement.accel.unwrap().y,
                measurement.accel.unwrap().z,
            ]);

            // print_state(&filter, time);
            filter.predict(ctrl_input);
            // print_state(&filter, time);
            filter.update(&meas);

            let state = filter.state();
            let pos = Vec3A::new(
                state[state_constants::POS_X],
                state[state_constants::POS_Y],
                state[state_constants::POS_Z],
            );
            let error = (pos - ground_truth).length();
            norm_acc += error;

            print_state(&filter, time);

            time += STEP_DURATION;
        }

        panic!("{:?}", norm_acc);
    }

    /// Prints the current state and covariance.
    pub fn print_state(filter: &Filter, time: f32) {
        let state = filter.state();
        let cov = filter.covariance();
        let std_x = cov[(state_constants::POS_X, state_constants::POS_X)].sqrt();
        let std_y = cov[(state_constants::POS_Y, state_constants::POS_Y)].sqrt();
        let std_z = cov[(state_constants::POS_Z, state_constants::POS_Z)].sqrt();
        let std_vx = cov[(state_constants::VEL_X, state_constants::VEL_X)].sqrt();
        let std_vy = cov[(state_constants::VEL_Y, state_constants::VEL_Y)].sqrt();
        let std_vz = cov[(state_constants::VEL_Z, state_constants::VEL_Z)].sqrt();
        // let std_ax = cov[(state_constants::ACC_X, state_constants::ACC_X)].sqrt();
        // let std_ay = cov[(state_constants::ACC_Y, state_constants::ACC_Y)].sqrt();
        // let std_az = cov[(state_constants::ACC_Z, state_constants::ACC_Z)].sqrt();
        let std_abx = cov[(state_constants::BIAS_ACC_X, state_constants::BIAS_ACC_X)].sqrt();
        let std_aby = cov[(state_constants::BIAS_ACC_Y, state_constants::BIAS_ACC_Y)].sqrt();
        let std_abz = cov[(state_constants::BIAS_ACC_Z, state_constants::BIAS_ACC_Z)].sqrt();
        println!(
            "t={:.2} s\n      x={:.2} ± {:.4} m\n      y={:.2} ± {:.4} m\n      z={:.2} ± {:.4} m\n     vx={:.2} ± {:.4} m/s\n     vy={:.2} ± {:.4} m/s\n     vz={:.2} ± {:.4} m/s\n     abx={:.2} ± {:.4} m/s^2\n     aby={:.2} ± {:.4} m/s^2\n     abz={:.2} ± {:.4} m/s^2",
            time,
            state[state_constants::POS_X], std_x,
            state[state_constants::POS_Y], std_y,
            state[state_constants::POS_Z], std_z,
            state[state_constants::VEL_X], std_vx,
            state[state_constants::VEL_Y], std_vy,
            state[state_constants::VEL_Z], std_vz,
            // state[state_constants::ACC_X], std_ax,
            // state[state_constants::ACC_Y], std_ay,
            // state[state_constants::ACC_Z], std_az,
            state[state_constants::BIAS_ACC_X], std_abx,
            state[state_constants::BIAS_ACC_Y], std_aby,
            state[state_constants::BIAS_ACC_Z], std_abz
        );
    }
}
