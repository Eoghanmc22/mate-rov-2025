use bevy::{
    ecs::component::Component,
    reflect::{prelude::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use serde::{Deserialize, Serialize};

use crate::{adapters::serde::ReflectSerdeAdapter, ecs_sync::NetId};

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct Singleton;

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct Robot;

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct Surface;

// TODO: This could be changed to a unit struct that is added and removed from the robot entity
#[derive(
    Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Eq, Default,
)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub enum Armed {
    Armed,
    #[default]
    Disarmed,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Eq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
// #[deprecated]
pub struct RobotId(pub NetId);

// #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
// #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
// pub struct Statistics(pub StableHashMap<String, f32>);
