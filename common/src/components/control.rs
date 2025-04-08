use bevy::{
    ecs::component::Component,
    reflect::{Reflect, ReflectDeserialize, ReflectSerialize},
};
use glam::Quat;
use serde::{Deserialize, Serialize};

use crate::adapters::serde::ReflectSerdeAdapter;
use crate::types::units::Meters;

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct DepthTarget(pub Meters);

/// Desired up vector
#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct OrientationTarget(pub Quat);
