use adskalman::{ObservationModel, TransitionModelLinearNoControl};
use nalgebra::{Matrix, OMatrix, OVector, Owned};

use super::{meas_constants, state_constants, KalmanConfig, OS, R, SS};

pub struct ROVObservationModel {
    observation: Matrix<R, OS, SS, Owned<R, OS, SS>>,
    observation_transpose: Matrix<R, SS, OS, Owned<R, SS, OS>>,
    observation_covariance: Matrix<R, OS, OS, Owned<R, OS, OS>>,
}

impl ROVObservationModel {
    pub fn new(config: &KalmanConfig) -> Self {
        let mut observation = OMatrix::<R, OS, SS>::zeros();

        // observation[(meas_constants::DEPTH, state_constants::POS_Z)] = 1.0;

        observation[(meas_constants::POS_X, state_constants::POS_X)] = 0.0;
        observation[(meas_constants::POS_Y, state_constants::POS_Y)] = 0.0;
        observation[(meas_constants::POS_Z, state_constants::POS_Z)] = 0.0;

        // observation[(meas_constants::VEL_X, state_constants::VEL_X)] = 1.0;
        // observation[(meas_constants::VEL_Y, state_constants::VEL_Y)] = 1.0;
        // observation[(meas_constants::VEL_Z, state_constants::VEL_Z)] = 1.0;

        observation[(meas_constants::ACC_X, state_constants::ACC_X)] = 1.0;
        observation[(meas_constants::ACC_Y, state_constants::ACC_Y)] = 1.0;
        observation[(meas_constants::ACC_Z, state_constants::ACC_Z)] = 1.0;

        observation[(meas_constants::ACC_X, state_constants::BIAS_ACC_X)] = 1.0;
        observation[(meas_constants::ACC_Y, state_constants::BIAS_ACC_Y)] = 1.0;
        observation[(meas_constants::ACC_Z, state_constants::BIAS_ACC_Z)] = 1.0;

        let observation_transpose = observation.transpose();

        let mut observation_noise = OVector::<R, OS>::zeros();
        // observation_noise[meas_constants::DEPTH] = config.depth_noise;
        observation_noise[meas_constants::POS_X] = 1e9;
        observation_noise[meas_constants::POS_Y] = 1e9;
        observation_noise[meas_constants::POS_Z] = 1e9;
        // observation_noise[meas_constants::VEL_X] = config.velo_noise;
        // observation_noise[meas_constants::VEL_Y] = config.velo_noise;
        // observation_noise[meas_constants::VEL_Z] = config.velo_noise;
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
