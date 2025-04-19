mod control;
pub use control::*;

mod pid;
pub use pid::*;

mod power;
pub use power::*;

mod sensor;
pub use sensor::*;

mod motor;
pub use motor::*;

mod system_monitor;
use stable_hashmap::StableHashMap;
pub use system_monitor::*;

mod thruster;
pub use thruster::*;

use bevy::{
    app::App,
    ecs::component::Component,
    reflect::{std_traits::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use serde::{Deserialize, Serialize};

use crate::{
    adapters::serde::ReflectSerdeAdapter,
    ecs_sync::{AppReplicateExt, NetId},
};

macro_rules! components {
    ($($name:ident),*) => {
        pub fn register_components(app: &mut App) {
            $(
                app.replicate::<$name>();
            )*
        }
    }
}

// TODO: think of a way where its a compile error for a type to not be registered here
components! {
    // This file
    Singleton,
    Robot,
    Surface,
    Armed,
    RobotId,

    // control.rs
    DepthTarget,
    OrientationTarget,

    // motor.rs
    MotorCameraReference,
    Motors,
    MotorSignal,
    MotorSignalType,
    MotorRawSignalRange,
    MotorContributionMode,
    MotorTargets,
    MotorContribution,
    GenericMotorId,

    // pid.rs
    PidConfig,
    PidResult,

    // power.rs
    MeasuredVoltage,
    CurrentDraw,

    // sensor.rs
    Orientation,
    GyroMeasurement,
    AccelerometerMeasurement,
    MagnetometerMeasurement,
    DepthMeasurement,
    DepthSettings,
    TempertureMeasurement,
    Leak,
    CameraDefinition,


    // system_monitor.rs
    SystemProcesses,
    SystemLoadAverage,
    SystemNetworks,
    SystemCpuTotal,
    SystemCores,
    SystemMemory,
    SystemTemperatures,
    SystemDisks,
    SystemUptime,
    SystemOs,

    // thruster.rs/movement_api
    TargetMovement,
    ActualMovement,
    MovementContribution,
    MovementAxisMaximums,
    MovementCurrentCap,
    DisableMovementApi,

    // thruster.rs/thruster_api
    TargetForce,
    ActualForce,
    ThrusterDefinition,
    Thrusters,
    ThrustContribution,
    JerkLimit
}

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
#[deprecated]
pub struct RobotId(pub NetId);

// #[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
// #[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
// pub struct Statistics(pub StableHashMap<String, f32>);
