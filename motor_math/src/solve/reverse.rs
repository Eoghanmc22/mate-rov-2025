//! Desired Movement -> Motor Commands

use std::fmt::Debug;
use std::hash::Hash;

use nalgebra::{vector, Vector6};
use serde::{Deserialize, Serialize};
use stable_hashmap::StableHashMap;
use tracing::instrument;

use crate::{
    motor_preformance::{Interpolation, MotorData, MotorRecord},
    MotorConfig, Movement, Number,
};

type HashMap<K, V> = StableHashMap<K, V>;

#[instrument(level = "trace", skip(motor_config), ret)]
pub fn reverse_solve<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    movement: Movement<D>,
    motor_config: &MotorConfig<MotorId, D>,
) -> HashMap<MotorId, D> {
    let movement_vec = Vector6::from_iterator(
        [movement.force, movement.torque]
            .iter()
            .flat_map(|it| it.as_slice())
            .cloned(),
    );

    let forces = motor_config.pseudo_inverse.clone() * movement_vec;

    let mut motor_forces = HashMap::default();
    for ((motor_id, _motor), force) in motor_config
        .motors
        .iter()
        .zip(Vec::from(forces.data).into_iter())
    {
        motor_forces.insert(motor_id.clone(), force);
    }

    motor_forces
}

#[instrument(level = "trace", skip(motor_config, motor_data), ret)]
pub fn forces_to_cmds<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    forces: HashMap<MotorId, D>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
) -> HashMap<MotorId, MotorRecord<D>> {
    let mut motor_cmds = HashMap::default();
    for (motor_id, force) in forces {
        let motor = motor_config.motor(&motor_id).expect("Bad motor id");
        let data = motor_data.lookup_by_force(force, Interpolation::LerpDirection(motor.direction));

        motor_cmds.insert(motor_id.clone(), data);
    }

    motor_cmds
}

/// Does not preserve force ratios
/// Runs in constant time
#[instrument(level = "trace", skip(motor_config, motor_data), ret)]
pub fn clamp_amperage_fast<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    motor_cmds: HashMap<MotorId, MotorRecord<D>>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    amperage_cap: f32,
) -> HashMap<MotorId, MotorRecord<D>> {
    let amperage_total = motor_cmds.values().map(|it| it.current).sum::<D>();

    if amperage_total.re() <= amperage_cap {
        return motor_cmds;
    } else {
        // TODO remove?
        // println!("CURRENT LIMIT HIT");
    }

    let amperage_ratio = D::from(amperage_cap) / amperage_total;

    let mut adjusted_motor_cmds = HashMap::default();
    for (motor_id, data) in motor_cmds {
        let direction = motor_config
            .motor(&motor_id)
            .map(|it| it.direction)
            .unwrap_or(crate::Direction::Clockwise);

        let adjusted_current = data.current.copysign(data.force) * amperage_ratio;
        let data_adjusted =
            motor_data.lookup_by_current(adjusted_current, Interpolation::LerpDirection(direction));

        adjusted_motor_cmds.insert(motor_id.clone(), data_adjusted);
    }

    adjusted_motor_cmds
}

#[instrument(level = "trace", skip(motor_config, motor_data), ret)]
pub fn clamp_amperage<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    motor_cmds: HashMap<MotorId, MotorRecord<D>>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    amperage_cap: f32,
    epsilon: f32,
) -> HashMap<MotorId, MotorRecord<D>> {
    let amperage_total = motor_cmds.values().map(|it| it.current).sum::<D>();

    if amperage_total.re() <= amperage_cap {
        return motor_cmds;
    } else {
        // TODO remove?
        // println!("CURRENT LIMIT HIT");
    }

    let force_ratio =
        binary_search_force_ratio(&motor_cmds, motor_config, motor_data, amperage_cap, epsilon);

    let mut adjusted_motor_cmds = HashMap::default();
    for (motor_id, data) in motor_cmds {
        let direction = motor_config
            .motor(&motor_id)
            .map(|it| it.direction)
            .unwrap_or(crate::Direction::Clockwise);

        let force_current = data.force * force_ratio;
        let data_adjusted =
            motor_data.lookup_by_force(force_current, Interpolation::LerpDirection(direction));

        adjusted_motor_cmds.insert(motor_id.clone(), data_adjusted);
    }

    adjusted_motor_cmds
}

// TODO: Validate this is using dual numbers correctly
pub fn binary_search_force_ratio<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    motor_cmds: &HashMap<MotorId, MotorRecord<D>>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    amperage_cap: f32,
    epsilon: f32,
) -> D {
    let (mut lower_bound, mut lower_current) = (D::zero(), D::zero());
    let (mut upper_bound, mut upper_current) = (D::from(f32::INFINITY), D::from(f32::INFINITY));
    let mut mid = D::one();

    loop {
        let mid_current = motor_cmds
            .iter()
            .map(|(motor_id, data)| {
                let direction = motor_config
                    .motor(motor_id)
                    .map(|it| it.direction)
                    .unwrap_or(crate::Direction::Clockwise);

                // FIXME: old code of copying force's sign to its self is a no-op could be a bug
                // let adjusted_force = data.force.copysign(data.force) * mid;
                let adjusted_force = data.force * mid;
                let data = motor_data
                    .lookup_by_force(adjusted_force, Interpolation::LerpDirection(direction));

                data.current
            })
            .sum::<D>();

        if mid_current.re() == 0.0 {
            return D::one();
        }
        if (mid_current.re() - amperage_cap).abs() < epsilon {
            return mid;
        }

        if mid_current.re() >= amperage_cap {
            upper_bound = mid;
            upper_current = mid_current;
        } else {
            lower_bound = mid;
            lower_current = mid_current;
        }

        if upper_bound.re() == f32::INFINITY {
            mid *= D::from(amperage_cap) / mid_current;
            // mid *= 2.0;
        } else {
            let alpha = (D::from(amperage_cap) - lower_current) / (upper_current - lower_current);
            mid = upper_bound * alpha + lower_bound * (D::one() - alpha)
            // mid = upper_bound / 2.0 + lower_bound / 2.0
        }
    }
}

#[derive(Debug, Hash, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Serialize, Deserialize)]
pub enum Axis {
    X,
    Y,
    Z,
    XRot,
    YRot,
    ZRot,
}

impl Axis {
    pub fn movement<D: Number>(&self) -> Movement<D> {
        match self {
            Axis::X => Movement {
                force: vector![D::one(), D::zero(), D::zero()],
                torque: vector![D::zero(), D::zero(), D::zero()],
            },
            Axis::Y => Movement {
                force: vector![D::zero(), D::one(), D::zero()],
                torque: vector![D::zero(), D::zero(), D::zero()],
            },
            Axis::Z => Movement {
                force: vector![D::zero(), D::zero(), D::one()],
                torque: vector![D::zero(), D::zero(), D::zero()],
            },
            Axis::XRot => Movement {
                force: vector![D::zero(), D::zero(), D::zero()],
                torque: vector![D::one(), D::zero(), D::zero()],
            },
            Axis::YRot => Movement {
                force: vector![D::zero(), D::zero(), D::zero()],
                torque: vector![D::zero(), D::one(), D::zero()],
            },
            Axis::ZRot => Movement {
                force: vector![D::zero(), D::zero(), D::zero()],
                torque: vector![D::zero(), D::zero(), D::one()],
            },
        }
    }
}

pub fn axis_maximums<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    amperage_cap: f32,
    epsilon: f32,
) -> HashMap<Axis, D> {
    [
        Axis::X,
        Axis::Y,
        Axis::Z,
        Axis::XRot,
        Axis::YRot,
        Axis::ZRot,
    ]
    .into_iter()
    .map(|it| (it, it.movement::<D>()))
    .map(|(axis, movement)| {
        let initial = 25.0;

        let forces = reverse_solve(movement * initial.into(), motor_config);
        let cmds = forces_to_cmds(forces, motor_config, motor_data);
        let scale =
            binary_search_force_ratio(&cmds, motor_config, motor_data, amperage_cap, epsilon);

        let value = scale * initial;
        (axis, value)
    })
    .collect()
}
