use bevy_reflect::{Reflect, ReflectDeserialize, ReflectSerialize};
use nalgebra::Vector3;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use serde::{Deserialize, Serialize};

use crate::{utils::VectorTransform, MotorConfig, Number, Thruster};

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
pub enum X3dMotorId {
    FrontRightTop,
    FrontRightBottom,
    FrontLeftTop,
    FrontLeftBottom,
    BackRightTop,
    BackRightBottom,
    BackLeftTop,
    BackLeftBottom,
}

impl<D: Number> MotorConfig<X3dMotorId, D> {
    pub fn new(front_right_top: Thruster<D>, center_mass: Vector3<D>) -> Self {
        #[rustfmt::skip]
        let motors = [
            (X3dMotorId::FrontRightTop, [].as_slice()),

            (X3dMotorId::FrontRightBottom, [VectorTransform::ReflectXY].as_slice()),
            (X3dMotorId::FrontLeftTop, [VectorTransform::ReflectYZ].as_slice()),
            (X3dMotorId::BackRightTop, [VectorTransform::ReflectXZ].as_slice()),

            (X3dMotorId::FrontLeftBottom, [VectorTransform::ReflectXY, VectorTransform::ReflectYZ].as_slice()),
            (X3dMotorId::BackLeftTop, [VectorTransform::ReflectYZ, VectorTransform::ReflectXZ].as_slice()),
            (X3dMotorId::BackRightBottom, [VectorTransform::ReflectXZ, VectorTransform::ReflectXY].as_slice()),

            (X3dMotorId::BackLeftBottom, [VectorTransform::ReflectXY, VectorTransform::ReflectYZ, VectorTransform::ReflectXZ].as_slice()),
        ];

        let motors = motors.into_iter().map(|(motor_id, transforms)| {
            let (position, orientation) = transforms.iter().fold(
                (front_right_top.position, front_right_top.orientation),
                |(position, orientation), transform| {
                    (
                        transform.transform(position),
                        transform.transform(orientation),
                    )
                },
            );

            (
                motor_id,
                Thruster {
                    position,
                    orientation,
                    direction: front_right_top.direction.flip_n(transforms.len() as _),
                },
            )
        });

        Self::new_raw(motors, center_mass)
    }
}
