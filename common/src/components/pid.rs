use bevy::{
    ecs::component::Component,
    reflect::{prelude::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use serde::{Deserialize, Serialize};

use crate::adapters::serde::ReflectSerdeAdapter;

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub i_zone: f32,
    pub max_integral: f32,
    pub max_output: f32,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct PidResult {
    pub error: f32,
    pub p: f32,
    pub i: f32,
    pub d: f32,

    pub correction: f32,
}
