use bevy::{
    ecs::component::Component,
    reflect::{Reflect, ReflectDeserialize, ReflectSerialize},
};
use serde::{Deserialize, Serialize};

use crate::adapters::serde::ReflectSerdeAdapter;
use crate::types::units::{Amperes, Volts};

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct MeasuredVoltage(pub Volts);

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct CurrentDraw(pub Amperes);
