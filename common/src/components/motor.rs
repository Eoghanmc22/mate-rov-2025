//! Servo and dc motor api
use std::{borrow::Cow, cmp::Ordering};

use bevy::{
    ecs::component::Component,
    reflect::{prelude::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use serde::{Deserialize, Serialize};
use stable_hashmap::StableHashMap;

use crate::adapters::serde::ReflectSerdeAdapter;

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct MotorCameraReference {
    pub camera: Cow<'static, str>,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct Motors {
    // TODO: Make ServoId type
    // TODO: Reevaluate if using Cow makes sense
    pub ids: Vec<GenericMotorId>,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub enum MotorSignal {
    Percent(f32),
    Raw(i32),
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub enum MotorSignalType {
    Position,
    Velocity,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct MotorRawSignalRange {
    pub min: i32,
    pub center: i32,
    pub max: i32,
}

impl MotorRawSignalRange {
    pub fn clamp_raw(&self, raw: i32) -> i32 {
        raw.clamp(self.min, self.max)
    }

    pub fn percent_from_raw(&self, raw: i32) -> f32 {
        match raw.cmp(&self.center) {
            Ordering::Greater => (raw - self.center) as f32 / (self.max - self.center) as f32,
            Ordering::Less => -(raw - self.center) as f32 / (self.min - self.center) as f32,
            Ordering::Equal => 0.0,
        }
    }

    pub fn raw_from_percent(&self, pct: f32) -> i32 {
        match pct.total_cmp(&0.0) {
            Ordering::Greater => (pct * (self.max - self.center) as f32) as i32 + self.center,
            Ordering::Less => (-pct * (self.min - self.center) as f32) as i32 + self.center,
            Ordering::Equal => self.center,
        }
    }
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub enum MotorContributionMode {
    // Uses contribution sum directly
    ZerothOrder,
    // Integrates contribution sum
    FirstOrder,
}

// NOTE: In the current impl, this only reflects non-thruster actuator targets, ie those controlled
// using the servo plugin
#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, /*Serialize, Deserialize,*/ Debug, PartialEq, Default)]
pub struct MotorTargets(pub StableHashMap<GenericMotorId, f32>);

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, /*Serialize, Deserialize,*/ Debug, PartialEq, Default)]
#[reflect(from_reflect = false)]
pub struct MotorContribution(pub StableHashMap<GenericMotorId, f32>);

#[derive(
    Component,
    Serialize,
    Deserialize,
    Reflect,
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct GenericMotorId(pub u8);
