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

components! {
    Singleton,
    Robot,
    Surface,
    Orientation,
    GyroMeasurement,
    AccelerometerMeasurement,
    MagnetometerMeasurement,
    TempertureMeasurement,
    DepthMeasurement,
    DepthTarget,
    DepthSettings,
    OrientationTarget,
    Leak,
    Armed,
    Camera,
    RobotId,
    Processes,
    LoadAverage,
    Networks,
    CpuTotal,
    Cores,
    Memory,
    Temperatures,
    Disks,
    Uptime,
    OperatingSystem,
    TargetForce,
    ActualForce,
    MotorTargets,
    ThrusterDefinition,
    ThrusterDefinition,
    MotorSignalType,
    Motors,
    Thrusters,
    TargetMovement,
    ActualMovement,
    MeasuredVoltage,
    MovementContribution,
    MotorContribution,
    ThrustContribution,
    MovementAxisMaximums,
    MovementCurrentCap,
    CurrentDraw,
    JerkLimit,
    PidConfig,
    PidResult
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
