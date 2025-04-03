use std::iter;

use bevy_reflect::{Reflect, ReflectDeserialize, ReflectSerialize};
use nalgebra::{vector, Vector3};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use serde::{Deserialize, Serialize};

use crate::{utils::VectorTransform, Motor, MotorConfig, Number};

/// Motor ids for blue rov heavy
#[derive(
    Clone,
    Copy,
    Debug,
    PartialOrd,
    Ord,
    PartialEq,
    Eq,
    Hash,
    IntoPrimitive,
    TryFromPrimitive,
    Serialize,
    Deserialize,
    Reflect,
)]
#[reflect(Serialize, Deserialize, Debug, PartialEq, Hash)]
#[repr(u8)]
pub enum BlueRovMotorId {
    LateralFrontLeft,
    LateralFrontRight,
    LateralBackLeft,
    LateralBackRight,
    VerticalRight,
    VerticalLeft,
}

impl<D: Number> MotorConfig<BlueRovMotorId, D> {
    pub fn new(
        lateral_front_right: Motor<D>,
        vertical_right: Motor<D>,
        center_mass: Vector3<D>,
    ) -> Self {
        #[rustfmt::skip]
        let motors = [
            // remove for two and three
            (BlueRovMotorId::LateralFrontRight, lateral_front_right, false, &[].as_slice()),
            (BlueRovMotorId::LateralFrontLeft, lateral_front_right, false, &[VectorTransform::ReflectYZ].as_slice()),
            (BlueRovMotorId::LateralBackRight, lateral_front_right, true, &[VectorTransform::ReflectXZ].as_slice()),
            // remove for two, and 6dof
            // (BlueRovMotorId::LateralBackLeft, lateral_front_right, true, &[VectorTransform::ReflectYZ, VectorTransform::ReflectXZ].as_slice()),

            (BlueRovMotorId::VerticalRight, vertical_right, false, &[].as_slice()),
            (BlueRovMotorId::VerticalLeft, vertical_right, true, &[VectorTransform::ReflectYZ].as_slice()),
        ];

        let motors = motors
            .into_iter()
            .map(|(motor_id, seed, direction, transforms)| {
                let (position, orientation) = transforms.iter().fold(
                    (seed.position, seed.orientation),
                    |(position, orientation), transform| {
                        (
                            transform.transform(position),
                            transform.transform(orientation),
                        )
                    },
                );

                (
                    motor_id,
                    Motor {
                        position,
                        orientation,
                        direction: seed.direction.flip_n(direction as _),
                    },
                )
            });

        let full_6dof = true;
        if !full_6dof {
            Self::new_raw(motors, center_mass)
        } else {
            Self::new_raw(
                motors.chain(iter::once((
                    BlueRovMotorId::LateralBackLeft,
                    Motor {
                        position: vector![
                            D::from(-0.4361 / 2.0),
                            D::from(-0.2400 / 2.0),
                            D::from(0.004)
                        ],
                        orientation: vector![D::zero(), D::zero(), D::one()],
                        direction: crate::Direction::CounterClockwise,
                    },
                ))),
                center_mass,
            )
        }
    }
}
