use bevy::{
    ecs::component::Component,
    reflect::{prelude::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use motor_math::{
    glam::{MovementGlam, ThrusterGlam},
    solve::reverse::Axis,
    ErasedMotorId, MotorConfig,
};
use serde::{Deserialize, Serialize};
use stable_hashmap::StableHashMap;

use crate::{
    adapters::serde::ReflectSerdeAdapter,
    types::units::{Amperes, Newtons},
};

pub use movement_api::*;
pub use thruster_api::*;

/// API for operating on the robot's net movement
mod movement_api {
    use glam::Vec3A;

    use super::*;

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct TargetMovement(pub MovementGlam);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct ActualMovement(pub MovementGlam);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
    pub struct MovementContribution(pub MovementGlam);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    // TODO: Store this as a MovementGlam
    pub struct MovementAxisMaximums(pub StableHashMap<Axis, Newtons>);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct MovementCurrentCap(pub Amperes);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct DisableMovementApi;

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct CenterOfMass(pub Vec3A);
}

/// API for operating on individual thrusters, mainly read only
mod thruster_api {
    use super::*;

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct TargetForce(pub Newtons);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct ActualForce(pub Newtons);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct ThrusterDefinition(pub ErasedMotorId, pub ThrusterGlam);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, /*Serialize, Deserialize,*/ Debug, PartialEq)]
    #[reflect(from_reflect = false)]
    pub struct Thrusters(pub MotorConfig<ErasedMotorId, motor_math::FloatType>);

    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
    #[reflect(SerdeAdapter, /*Serialize, Deserialize,*/ Debug, PartialEq, Default)]
    #[reflect(from_reflect = false)]
    pub struct ThrustContribution(pub StableHashMap<ErasedMotorId, Newtons>);

    /// Units of newtons per second
    #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    pub struct JerkLimit(pub f32);

    // Not Implemented
    // #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
    // #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
    // pub struct DisableThrusterApi;
}
