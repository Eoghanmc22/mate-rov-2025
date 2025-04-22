use crate::ecs_sync::AppReplicateExt;
use bevy::prelude::*;

macro_rules! components {
    ($($mod:ident :: { $($name:ident),* $(,)? }),* $(,)?) => {
        pub fn register_components(app: &mut App) {
            $(
                $(
                    app.replicate::<$name>();
                )*
            )*
        }

        $(
            mod $mod;
            $(
                pub use $mod::$name;
            )*
        )*
    }
}

components! {
    core::{
        Singleton,
        Robot,
        Surface,
        Armed,
        RobotId,
    },

    control::{
        DepthTarget,
        OrientationTarget,
    },

    motor::{
        MotorCameraReference,
        Motors,
        MotorSignal,
        MotorSignalType,
        MotorRawSignalRange,
        MotorContributionMode,
        MotorTargets,
        MotorSlewRate,
        MotorContribution,
        GenericMotorId,
    },

    pid::{
        PidConfig,
        PidResult,
    },

    power::{
        MeasuredVoltage,
        CurrentDraw,
    },

    sensor::{
        Orientation,
        GyroMeasurement,
        AccelerometerMeasurement,
        MagnetometerMeasurement,
        DepthMeasurement,
        DepthSettings,
        TempertureMeasurement,
        Leak,
        CameraDefinition,
    },

    system_monitor::{
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
    },

    thruster::{
        // Movement Api
        TargetMovement,
        ActualMovement,
        MovementContribution,
        MovementAxisMaximums,
        MovementCurrentCap,
        DisableMovementApi,

        // Thruster Api
        TargetForce,
        ActualForce,
        ThrusterDefinition,
        Thrusters,
        ThrustContribution,
        JerkLimit,
    },
}
