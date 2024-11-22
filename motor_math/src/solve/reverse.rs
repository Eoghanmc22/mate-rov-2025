//! Desired Movement -> Motor Commands

use core::f32;
use std::fmt::Debug;
use std::hash::Hash;

use bevy_reflect::Reflect;
use nalgebra::{vector, Vector6};
use serde::{Deserialize, Serialize};
use stable_hashmap::StableHashMap;
use tracing::{error, instrument, warn};

use crate::{
    motor_preformance::{Interpolation, MotorData, MotorRecord},
    solve::forward::forward_solve,
    FloatType, MotorConfig, Movement, Number,
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
    forces_to_cmds_impl(forces, motor_config, motor_data, false)
}

#[instrument(level = "trace", skip(motor_config, motor_data), ret)]
pub fn forces_to_cmds_extrapolated<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    forces: HashMap<MotorId, D>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
) -> HashMap<MotorId, MotorRecord<D>> {
    forces_to_cmds_impl(forces, motor_config, motor_data, true)
}

fn forces_to_cmds_impl<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    forces: HashMap<MotorId, D>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    extrapolate: bool,
) -> HashMap<MotorId, MotorRecord<D>> {
    let mut motor_cmds = HashMap::default();
    for (motor_id, force) in forces {
        let motor = motor_config.motor(&motor_id).expect("Bad motor id");
        let data = motor_data.lookup_by_force(
            force,
            Interpolation::LerpDirection(motor.direction),
            extrapolate,
        );

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
    amperage_cap: FloatType,
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
        let data_adjusted = motor_data.lookup_by_current(
            adjusted_current,
            Interpolation::LerpDirection(direction),
            false,
        );

        adjusted_motor_cmds.insert(motor_id.clone(), data_adjusted);
    }

    adjusted_motor_cmds
}

#[instrument(level = "trace", skip(motor_config, motor_data), ret)]
pub fn clamp_amperage<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    motor_cmds: HashMap<MotorId, MotorRecord<D>>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    amperage_cap: FloatType,
    epsilon: FloatType,
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
        let data_adjusted = motor_data.lookup_by_force(
            force_current,
            Interpolation::LerpDirection(direction),
            false,
        );

        adjusted_motor_cmds.insert(motor_id.clone(), data_adjusted);
    }

    adjusted_motor_cmds
}

/// Determines the ratio that `motor_cmds` would need to be multiplied by in order for the motors to use the largest fraction of the amperage_cap possible
// TODO: Validate this is using dual numbers correctly
pub fn binary_search_force_ratio<D: Number, MotorId: Hash + Ord + Clone + Debug>(
    motor_cmds: &HashMap<MotorId, MotorRecord<D>>,
    motor_config: &MotorConfig<MotorId, D>,
    motor_data: &MotorData,
    mut amperage_cap: FloatType,
    epsilon: FloatType,
) -> D {
    let (mut lower_bound, mut lower_current) = (D::zero(), D::zero());
    let (mut upper_bound, mut upper_current) =
        (D::from(FloatType::INFINITY), D::from(FloatType::INFINITY));
    let mut mid = D::one();

    let mut max_iters = 15;
    let mut learn_cap = false;

    loop {
        // Determine the current the current value of mid would draw
        // Returns `mid_force` and `expected_force` for the motor where the difference is largest
        let (mid_current, mid_force, expected_force, delta_force) = motor_cmds
            .iter()
            .map(|(motor_id, data)| {
                // Determine motor spin direction
                let direction = motor_config
                    .motor(motor_id)
                    .map(|it| it.direction)
                    .unwrap_or(crate::Direction::Clockwise);

                // Calculate target force
                let adjusted_force = coerce_zero(data.force, epsilon) * mid;

                // Lookup spline point for the target force
                let data = motor_data.lookup_by_force(
                    adjusted_force,
                    Interpolation::LerpDirection(direction),
                    false,
                );

                // `data.force` will be different from `adjusted_force` in the case where
                // `adjusted_force` is greater than the max the motor is able to produce

                (
                    // The current used by this motor
                    coerce_zero(data.current.abs(), epsilon),
                    // The force the motor will produce
                    coerce_zero(data.force.abs(), epsilon),
                    // The force we wanted the motor produce
                    adjusted_force.abs(),
                )
            })
            // (mid_current, mid_force, expected_force, delta_force)
            .fold((D::zero(), D::zero(), D::zero(), D::zero()), |acc, it| {
                // Calculate the difference between the requested and actual force
                let delta = (it.2 - it.1).abs();

                // Sum the current, and if this is the worst motor so far, replace the preavious force values with those from this motor
                if delta > acc.3 {
                    // Delta is worse, replace force data with new values
                    (acc.0 + it.0, it.1, it.2, delta)
                } else {
                    // Only sum the current and preserve existing force values
                    (acc.0 + it.0, acc.1, acc.2, acc.3)
                }
            });

        if mid_current.re() == 0.0 {
            return D::zero();
        }

        // Prevents the force ratio from diverging when it is impossible to reach the input
        // amperage cap. This happens when `amperage_cap` is greater than the max current the
        // motors can draw
        if delta_force.re().abs() > epsilon {
            // Should be unreachable
            if learn_cap {
                error!("Reached potantial loop condition in binary_search_force_ratio")
            }

            // TODO: Is this correct?
            (lower_bound, lower_current) = (D::zero(), D::zero());
            (upper_bound, upper_current) = (mid, mid_current);

            // We need to update amperage_cap to be no larger than the current used by the new
            // value of mid, but that information isnt avaible yet. Set a flag to do this on the
            // next cycle
            learn_cap = true;

            // Calculated a new value of mid such that force produced is exactly equal to the max
            // force the motors are capaible of
            mid *= mid_force / expected_force;

            // Jump back to the start of the loop to recompute mid_current based on the new mid
            continue;
        } else if learn_cap {
            // Update the amperage cap to match the motor max current
            amperage_cap = amperage_cap.min(mid_current.re());
            learn_cap = false;
        }

        // Handles normal case
        if (mid_current.re() - amperage_cap).abs() < epsilon {
            return mid;
        }

        // Updates upper and lower bound based on observation
        if mid_current.re() >= amperage_cap {
            upper_bound = mid;
            upper_current = mid_current;
        } else {
            lower_bound = mid;
            lower_current = mid_current;
        }

        // Determines next test point based on the new bounds
        if upper_bound.re() == FloatType::INFINITY {
            mid *= D::from(amperage_cap) / mid_current;
        } else {
            let alpha = (D::from(amperage_cap) - lower_current) / (upper_current - lower_current);
            mid = upper_bound * alpha + lower_bound * (D::one() - alpha)
        }

        // Upper limit on number of iterations
        // Prevents infinite looping
        max_iters -= 1;
        if max_iters == 0 {
            warn!("Hit max iters on binary_search_force_ratio");
            return mid;
        }
    }
}

#[derive(
    Debug, Hash, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Serialize, Deserialize, Reflect,
)]
#[reflect(Debug, PartialEq)]
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
    amperage_cap: FloatType,
    epsilon: FloatType,
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
    .map(|(axis, mut movement)| {
        // Must be less than the smallest expected strength
        let guess_magnitude = 15.0;
        movement *= guess_magnitude.into();

        let forces = reverse_solve(movement, motor_config);

        // TODO: Is this needed?
        // let cmds = dbg!(forces_to_cmds(forces, motor_config, motor_data));
        // let forces = cmds
        //     .iter()
        //     .map(|(motor, data)| (motor.clone(), data.force))
        //     .collect();

        let actual_movement = forward_solve(motor_config, &forces);

        let actual_magnitude = actual_movement.force.dot(&movement.force).re().sqrt()
            + actual_movement.torque.dot(&movement.torque).re().sqrt();

        if (actual_magnitude - guess_magnitude).abs() < epsilon {
            let cmds = forces_to_cmds_extrapolated(forces, motor_config, motor_data);
            let scale =
                binary_search_force_ratio(&cmds, motor_config, motor_data, amperage_cap, epsilon);

            let value = scale * guess_magnitude;

            (axis, value)
        } else {
            (axis, D::zero())
        }
    })
    .collect()
}

fn coerce_zero<D: Number>(value: D, epsilon: FloatType) -> D {
    if value.re().abs() < epsilon {
        return D::zero();
    }

    value
}
