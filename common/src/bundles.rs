use bevy::{core::Name, ecs::bundle::Bundle, transform::components::Transform};

use crate::components::{
    AccelerometerMeasurement, ActualForce, ActualMovement, Armed, CameraDefinition, CurrentDraw,
    DepthMeasurement, GenericMotorId, GyroMeasurement, Leak, MagnetometerMeasurement,
    MeasuredVoltage, MotorContributionMode, MotorRawSignalRange, MotorSignal, MotorSignalType,
    MovementAxisMaximums, MovementContribution, MovementCurrentCap, Orientation, Robot, RobotId,
    SystemCores, SystemCpuTotal, SystemDisks, SystemLoadAverage, SystemMemory, SystemNetworks,
    SystemOs, SystemProcesses, SystemTemperatures, SystemUptime, TargetForce, TargetMovement,
    TempertureMeasurement, ThrusterDefinition, Thrusters,
};

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct RobotBundle {
    pub core: RobotCoreBundle,
    pub sensors: RobotSensorBundle,
    pub system: RobotSystemBundle,
    pub actuators: RobotThrusterBundle,
    pub power: RobotPowerBundle,
    // pub manual: Option<PwmManualControl>,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct RobotCoreBundle {
    pub marker: Robot,
    // pub status: RobotStatus,
    pub name: Name,

    pub robot_id: RobotId,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct RobotSensorBundle {
    pub orientation: Orientation,
    pub gyro: GyroMeasurement,
    pub accel: AccelerometerMeasurement,
    pub mag: MagnetometerMeasurement,
    pub depth: DepthMeasurement,
    pub temp: TempertureMeasurement,
    pub leak: Leak,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct RobotSystemBundle {
    pub processes: SystemProcesses,
    pub load_average: SystemLoadAverage,
    pub networks: SystemNetworks,
    pub cpu: SystemCpuTotal,
    pub cores: SystemCores,
    pub memory: SystemMemory,
    pub temps: SystemTemperatures,
    pub disks: SystemDisks,
    pub uptime: SystemUptime,
    pub os: SystemOs,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct RobotThrusterBundle {
    pub movement_target: TargetMovement,
    pub movement_actual: ActualMovement,

    pub thruster_config: Thrusters,
    // pub motor_config: Motors,
    pub axis_maximums: MovementAxisMaximums,
    pub current_cap: MovementCurrentCap,

    pub armed: Armed,
}

// TODO(mid): Sensor not implemented
#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct RobotPowerBundle {
    pub voltage: MeasuredVoltage,
    pub current_draw: CurrentDraw,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct CameraBundle {
    pub name: Name,
    pub camera: CameraDefinition,
    pub transform: Transform,

    pub robot: RobotId,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct ThrusterBundle {
    pub actuator: ActuatorBundle,
    pub motor: ThrusterDefinition,

    pub target_force: TargetForce,
    pub actual_force: ActualForce,
    pub current_draw: CurrentDraw,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct MotorBundle {
    pub actuator: ActuatorBundle,
    // pub servo: ThrusterDefinition,
    pub mode: MotorContributionMode,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct ActuatorBundle {
    pub name: Name,
    pub channel: GenericMotorId,
    pub signal: MotorSignal,
    pub signal_type: MotorSignalType,
    pub signal_range: MotorRawSignalRange,
    // pub camera_ref: Option<MotorCameraReference>,
    pub robot: RobotId,
}

#[derive(Bundle, PartialEq)]
#[deprecated]
pub struct MovementContributionBundle {
    pub name: Name,

    pub contribution: MovementContribution,

    pub robot: RobotId,
}
