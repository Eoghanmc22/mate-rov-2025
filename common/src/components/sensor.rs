use std::net::SocketAddr;

use bevy::{
    ecs::component::Component,
    reflect::{prelude::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use glam::Quat;
use serde::{Deserialize, Serialize};

use crate::{
    adapters::serde::ReflectSerdeAdapter,
    types::units::{Celsius, Dps, GForce, Gauss, Mbar, Meters},
};
#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct Orientation(pub Quat);

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct GyroMeasurement {
    // TODO: Consider switching to radians
    pub x: Dps,
    pub y: Dps,
    pub z: Dps,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct AccelerometerMeasurement {
    // TODO: Consider switching to m/s^2
    pub x: GForce,
    pub y: GForce,
    pub z: GForce,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct MagnetometerMeasurement {
    // TODO: Determine if this is the most appreciate unit
    pub x: Gauss,
    pub y: Gauss,
    pub z: Gauss,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct DepthMeasurement {
    pub depth: Meters,
    pub altitude: Meters,
    pub pressure: Mbar,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct DepthSettings {
    pub sea_level: Mbar,
    pub fluid_density: f32,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct TempertureMeasurement {
    pub temperature: Celsius,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct Leak(pub bool);

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Eq)]
#[reflect(from_reflect = false)]
#[reflect(SerdeAdapter, /*Serialize, Deserialize,*/ Debug, PartialEq)]
pub struct CameraDefinition {
    // TODO(low): This bad
    #[reflect(ignore)]
    pub location: SocketAddr,
}
