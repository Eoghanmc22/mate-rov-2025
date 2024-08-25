//! Motor Commands -> Movement

use ahash::HashMap;
use nalgebra::DVector;
use std::{fmt::Debug, hash::Hash};
use tracing::instrument;

use crate::{MotorConfig, Movement, Number};

#[instrument(level = "trace", skip(motor_config), ret)]
pub fn forward_solve<D: Number, MotorId: Hash + Ord + Debug>(
    motor_config: &MotorConfig<MotorId, D>,
    motor_forces: &HashMap<MotorId, D>,
) -> Movement<D> {
    let force_vec = DVector::from_iterator(
        motor_config.motors.len(),
        motor_config
            .motors()
            .map(|(id, _motor)| motor_forces.get(id).cloned().unwrap_or(D::zero())),
    );

    let movement = motor_config.matrix.clone() * force_vec;
    let force = movement.fixed_rows::<3>(0);
    let torque = movement.fixed_rows::<3>(3);

    Movement {
        force: force.into(),
        torque: torque.into(),
    }
}
