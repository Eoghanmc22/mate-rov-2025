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
            position: vec3a(position.x.re(), position.y.re(), position.z.re()),
            orientation: vec3a(orientation.x.re(), orientation.y.re(), orientation.z.re()),
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
                N::from(position.x),
                N::from(position.y),
                N::from(position.z)
            ),
            orientation: vector!(
                N::from(orientation.x),
                N::from(orientation.y),
                N::from(orientation.z)
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
            position: vec3a(position.x.re(), position.y.re(), position.z.re()),
            orientation: vec3a(orientation.x.re(), orientation.y.re(), orientation.z.re()),
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
                N::from(position.x),
                N::from(position.y),
                N::from(position.z)
            ),
            orientation: vector!(
                N::from(orientation.x),
                N::from(orientation.y),
                N::from(orientation.z)
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
            force: vec3a(force.x.re(), force.y.re(), force.z.re()),
            torque: vec3a(torque.x.re(), torque.y.re(), torque.z.re()),
        }
    }
}

impl<N: Number> From<MovementGlam> for Movement<N> {
    fn from(value: MovementGlam) -> Self {
        let MovementGlam { force, torque } = value;
        Movement {
            force: vector!(N::from(force.x), N::from(force.y), N::from(force.z)),
            torque: vector!(N::from(torque.x), N::from(torque.y), N::from(torque.z)),
        }
    }
}

impl<N: Number + Copy> From<&Movement<N>> for MovementGlam {
    fn from(value: &Movement<N>) -> Self {
        let Movement { force, torque } = *value;
        MovementGlam {
            force: vec3a(force.x.re(), force.y.re(), force.z.re()),
            torque: vec3a(torque.x.re(), torque.y.re(), torque.z.re()),
        }
    }
}

impl<N: Number> From<&MovementGlam> for Movement<N> {
    fn from(value: &MovementGlam) -> Self {
        let MovementGlam { force, torque } = *value;
        Movement {
            force: vector!(N::from(force.x), N::from(force.y), N::from(force.z)),
            torque: vector!(N::from(torque.x), N::from(torque.y), N::from(torque.z)),
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
