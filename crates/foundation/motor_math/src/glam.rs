use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

use bevy_reflect::{Reflect, ReflectDeserialize, ReflectSerialize};
use glam::{vec3a, Vec3A};
use nalgebra::vector;
use serde::{Deserialize, Serialize};

use crate::{Direction, Motor, Movement, Number};

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Reflect, PartialEq)]
#[reflect(Serialize, Deserialize, Debug, PartialEq)]
pub struct MotorGlam {
    /// Offset from origin
    pub position: Vec3A,
    /// Unit vector
    pub orientation: Vec3A,

    pub direction: Direction,
}

impl<N: Number> From<Motor<N>> for MotorGlam {
    fn from(value: Motor<N>) -> Self {
        let Motor {
            position,
            orientation,
            direction,
        } = value;
        MotorGlam {
            position: vec3a(
                position.x.re() as _,
                position.y.re() as _,
                position.z.re() as _,
            ),
            orientation: vec3a(
                orientation.x.re() as _,
                orientation.y.re() as _,
                orientation.z.re() as _,
            ),
            direction,
        }
    }
}

impl<N: Number> From<MotorGlam> for Motor<N> {
    fn from(value: MotorGlam) -> Self {
        let MotorGlam {
            position,
            orientation,
            direction,
        } = value;
        Motor {
            position: vector!(
                N::from(position.x as _),
                N::from(position.y as _),
                N::from(position.z as _)
            ),
            orientation: vector!(
                N::from(orientation.x as _),
                N::from(orientation.y as _),
                N::from(orientation.z as _)
            ),
            direction,
        }
    }
}

impl<N: Number + Copy> From<&Motor<N>> for MotorGlam {
    fn from(value: &Motor<N>) -> Self {
        let Motor {
            position,
            orientation,
            direction,
        } = *value;
        MotorGlam {
            position: vec3a(
                position.x.re() as _,
                position.y.re() as _,
                position.z.re() as _,
            ),
            orientation: vec3a(
                orientation.x.re() as _,
                orientation.y.re() as _,
                orientation.z.re() as _,
            ),
            direction,
        }
    }
}

impl<N: Number> From<&MotorGlam> for Motor<N> {
    fn from(value: &MotorGlam) -> Self {
        let MotorGlam {
            position,
            orientation,
            direction,
        } = *value;
        Motor {
            position: vector!(
                N::from(position.x as _),
                N::from(position.y as _),
                N::from(position.z as _)
            ),
            orientation: vector!(
                N::from(orientation.x as _),
                N::from(orientation.y as _),
                N::from(orientation.z as _)
            ),
            direction,
        }
    }
}

#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize, Reflect, PartialEq)]
#[reflect(Debug, PartialEq)]
pub struct MovementGlam {
    pub force: Vec3A,
    pub torque: Vec3A,
}

impl<N: Number> From<Movement<N>> for MovementGlam {
    fn from(value: Movement<N>) -> Self {
        let Movement { force, torque } = value;
        MovementGlam {
            force: vec3a(force.x.re() as _, force.y.re() as _, force.z.re() as _),
            torque: vec3a(torque.x.re() as _, torque.y.re() as _, torque.z.re() as _),
        }
    }
}

impl<N: Number> From<MovementGlam> for Movement<N> {
    fn from(value: MovementGlam) -> Self {
        let MovementGlam { force, torque } = value;
        Movement {
            force: vector!(
                N::from(force.x as _),
                N::from(force.y as _),
                N::from(force.z as _)
            ),
            torque: vector!(
                N::from(torque.x as _),
                N::from(torque.y as _),
                N::from(torque.z as _)
            ),
        }
    }
}

impl<N: Number + Copy> From<&Movement<N>> for MovementGlam {
    fn from(value: &Movement<N>) -> Self {
        let Movement { force, torque } = *value;
        MovementGlam {
            force: vec3a(force.x.re() as _, force.y.re() as _, force.z.re() as _),
            torque: vec3a(torque.x.re() as _, torque.y.re() as _, torque.z.re() as _),
        }
    }
}

impl<N: Number> From<&MovementGlam> for Movement<N> {
    fn from(value: &MovementGlam) -> Self {
        let MovementGlam { force, torque } = *value;
        Movement {
            force: vector!(
                N::from(force.x as _),
                N::from(force.y as _),
                N::from(force.z as _)
            ),
            torque: vector!(
                N::from(torque.x as _),
                N::from(torque.y as _),
                N::from(torque.z as _)
            ),
        }
    }
}

impl Add for MovementGlam {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            force: self.force + rhs.force,
            torque: self.torque + rhs.torque,
        }
    }
}

impl AddAssign for MovementGlam {
    fn add_assign(&mut self, rhs: Self) {
        self.force += rhs.force;
        self.torque += rhs.torque;
    }
}

impl Sub for MovementGlam {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            force: self.force - rhs.force,
            torque: self.torque - rhs.torque,
        }
    }
}

impl SubAssign for MovementGlam {
    fn sub_assign(&mut self, rhs: Self) {
        self.force -= rhs.force;
        self.torque -= rhs.torque;
    }
}

impl Mul<f32> for MovementGlam {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            force: self.force * rhs,
            torque: self.torque * rhs,
        }
    }
}

impl MulAssign<f32> for MovementGlam {
    fn mul_assign(&mut self, rhs: f32) {
        self.force *= rhs;
        self.torque *= rhs;
    }
}

impl Div<f32> for MovementGlam {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            force: self.force / rhs,
            torque: self.torque / rhs,
        }
    }
}

impl DivAssign<f32> for MovementGlam {
    fn div_assign(&mut self, rhs: f32) {
        self.force /= rhs;
        self.torque /= rhs;
    }
}
