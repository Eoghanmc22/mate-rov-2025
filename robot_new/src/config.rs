pub mod camera;
pub mod control;
pub mod interfaces;
pub mod servo;
pub mod thruster;

use bevy::ecs::system::Resource;
use serde::{Deserialize, Serialize};

use crate::config::{
    camera::CameraDefinition,
    control::ControlSystemDefinition,
    interfaces::InterfaceDefinition,
    servo::ServoDefinition,
    thruster::{ThrusterConfigDefinition, ThrusterDefinition},
};

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub robot: RobotDefinition,
    pub interfaces: Vec<InterfaceDefinition>,
    pub thruster_config: ThrusterConfigDefinition,
    pub thrusters: Vec<ThrusterDefinition>,
    // TODO: Rename, Type name used elsewhere
    pub servos: Vec<ServoDefinition>,
    pub cameras: Vec<CameraDefinition>,
    pub control: ControlSystemDefinition,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotDefinition {
    pub name: String,
    pub port: u16,
}
