#![feature(test)]

// +X: Right, +Y: Forwards, +Z: Up
// +XR: Pitch Up, +YR: Roll Clockwise, +ZR: Yaw Counter Clockwise (top view)

pub mod blue_rov;
pub mod blue_rov_heavy;
#[cfg(feature = "glam")]
pub mod glam;
pub mod motor_preformance;
pub mod solve;
pub mod utils;
pub mod x3d;

use std::{
    fmt::Debug,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

use bevy_reflect::prelude::ReflectDefault;
use bevy_reflect::{Reflect, ReflectDeserialize, ReflectSerialize};
use nalgebra::{Matrix6xX, MatrixXx6, RealField, Vector3, SVD};
use num_dual::DualNum;
use serde::{Deserialize, Serialize};
use tracing::instrument;

#[cfg(feature = "double_precision")]
pub type FloatType = f64;

#[cfg(not(feature = "double_precision"))]
pub type FloatType = f32;

// Should be implemented for f32 and f32 backed num-dual types
pub trait Number: DualNum<FloatType> + RealField + Debug + Copy {}
impl<T> Number for T where T: DualNum<FloatType> + RealField + Debug + Copy {}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Reflect)]
#[reflect(from_reflect = false)]
#[reflect(Debug, PartialEq)]
pub struct MotorConfig<MotorId: Debug + Ord, D: Number> {
    pub motors: Vec<(MotorId, Motor<D>)>,
    #[reflect(ignore)]
    pub matrix: Matrix6xX<D>,
    #[reflect(ignore)]
    pub pseudo_inverse: MatrixXx6<D>,
}

impl<MotorId: Ord + Debug, D: Number> MotorConfig<MotorId, D> {
    #[instrument(level = "trace", skip_all, ret)]
    pub fn new_raw(
        motors: impl IntoIterator<Item = (MotorId, Motor<D>)>,
        center_mass: Vector3<D>,
    ) -> Self {
        let mut motors: Vec<_> = motors
            .into_iter()
            .map(|mut it| {
                let norm = it.1.orientation.norm();
                if norm.re() != 0.0 {
                    it.1.orientation.unscale_mut(norm);
                }

                it
            })
            .collect();
        motors.sort_by(|a, b| MotorId::cmp(&a.0, &b.0));
        motors.dedup_by(|a, b| a.0 == b.0);

        // TODO: There has to be a better way
        let matrix = Matrix6xX::<D>::from_iterator(
            motors.len(),
            motors.iter().flat_map(|(_id, motor)| {
                let force = motor.orientation;
                let torque = (motor.position - center_mass).cross(&motor.orientation);

                [force, torque]
                    .into_iter()
                    .flat_map(|it| it.data.0.into_iter().flatten())
            }),
        );

        let svd = SVD::try_new_unordered(
            matrix.clone(),
            true,
            true,
            // D::RealField::default_epsilon() * 5.0,
            D::from(1e-5),
            100,
        )
        .unwrap_or_else(|| {
            panic!(
                "Could not find the pseudo inverse for {}",
                matrix.map(|it| it.re())
            )
        });
        let pseudo_inverse = svd.pseudo_inverse(D::from(0.0001)).unwrap();

        Self {
            motors,
            matrix,
            pseudo_inverse,
        }
    }

    pub fn motor(&self, motor: &MotorId) -> Option<&Motor<D>> {
        // self.motors.get(motor)
        self.motors.iter().find(|it| &it.0 == motor).map(|it| &it.1)
    }

    pub fn motors(&self) -> impl Iterator<Item = (&MotorId, &Motor<D>)> {
        self.motors.iter().map(|it| (&it.0, &it.1))
    }
}

pub type ErasedMotorId = u8;

impl<MotorId: Ord + Debug + Into<ErasedMotorId> + Clone, D: Number> MotorConfig<MotorId, D> {
    /// Order of ErasedMotorIds must match the order of MotorId given by the ord trait
    pub fn erase(self) -> MotorConfig<ErasedMotorId, D> {
        let MotorConfig {
            motors,
            matrix,
            pseudo_inverse,
        } = self;

        let motors = motors
            .into_iter()
            .map(|(id, motor)| (id.into(), motor))
            .collect();

        MotorConfig {
            motors,
            matrix,
            pseudo_inverse,
        }
    }
}

impl<D: Number> MotorConfig<ErasedMotorId, D> {
    /// Order of ErasedMotorIds must match the order of MotorId given by the ord trait
    pub fn unerase<MotorId: Ord + Debug + TryFrom<ErasedMotorId>>(
        self,
    ) -> Result<MotorConfig<MotorId, D>, <MotorId as TryFrom<ErasedMotorId>>::Error> {
        let MotorConfig {
            motors,
            matrix,
            pseudo_inverse,
        } = self;

        let motors = motors
            .into_iter()
            .map(|(id, motor)| MotorId::try_from(id).map(|it| (it, motor)))
            .collect::<Result<_, _>>()?;

        Ok(MotorConfig {
            motors,
            matrix,
            pseudo_inverse,
        })
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Reflect, PartialEq)]
#[reflect(Debug, PartialEq, Default)]
pub struct Motor<D: Number> {
    /// Offset from origin
    #[reflect(ignore)]
    pub position: Vector3<D>,
    /// Unit vector
    #[reflect(ignore)]
    pub orientation: Vector3<D>,

    pub direction: Direction,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Reflect, PartialEq, Eq)]
#[reflect(Serialize, Deserialize, Debug, PartialEq)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}

impl Direction {
    pub fn get_sign(&self) -> f32 {
        match self {
            Direction::Clockwise => 1.0,
            Direction::CounterClockwise => -1.0,
        }
    }

    pub fn from_sign(sign: f32) -> Self {
        if sign.signum() == 1.0 {
            Direction::Clockwise
        } else {
            Direction::CounterClockwise
        }
    }

    pub fn flip_n(&self, count: i32) -> Self {
        let sign = self.get_sign();
        let new_sign = sign * (-1.0f32).powi(count);
        Self::from_sign(new_sign)
    }
}

impl<D: Number> Default for Motor<D> {
    fn default() -> Self {
        Self {
            position: Vector3::<D>::zeros(),
            orientation: Vector3::<D>::zeros(),
            direction: Direction::Clockwise,
        }
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Reflect, PartialEq)]
#[reflect(Debug, PartialEq, Default)]
pub struct Movement<D: Number> {
    #[reflect(ignore)]
    pub force: Vector3<D>,
    #[reflect(ignore)]
    pub torque: Vector3<D>,
}

impl<D: Number> Default for Movement<D> {
    fn default() -> Self {
        Self {
            force: Vector3::<D>::zeros(),
            torque: Vector3::<D>::zeros(),
        }
    }
}

impl<D: Number> Add for Movement<D> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            force: self.force + rhs.force,
            torque: self.torque + rhs.torque,
        }
    }
}

impl<D: Number> AddAssign for Movement<D> {
    fn add_assign(&mut self, rhs: Self) {
        self.force += rhs.force;
        self.torque += rhs.torque;
    }
}

impl<D: Number> Sub for Movement<D> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            force: self.force - rhs.force,
            torque: self.torque - rhs.torque,
        }
    }
}

impl<D: Number> SubAssign for Movement<D> {
    fn sub_assign(&mut self, rhs: Self) {
        self.force -= rhs.force;
        self.torque -= rhs.torque;
    }
}

impl<D: Number> Mul<D> for Movement<D> {
    type Output = Self;

    fn mul(self, rhs: D) -> Self::Output {
        Self {
            force: self.force * rhs,
            torque: self.torque * rhs,
        }
    }
}

impl<D: Number> MulAssign<D> for Movement<D> {
    fn mul_assign(&mut self, rhs: D) {
        self.force *= rhs;
        self.torque *= rhs;
    }
}

impl<D: Number> Div<D> for Movement<D> {
    type Output = Self;

    fn div(self, rhs: D) -> Self::Output {
        Self {
            force: self.force / rhs,
            torque: self.torque / rhs,
        }
    }
}

impl<D: Number> DivAssign<D> for Movement<D> {
    fn div_assign(&mut self, rhs: D) {
        self.force /= rhs;
        self.torque /= rhs;
    }
}
