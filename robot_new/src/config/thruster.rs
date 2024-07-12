use std::path::PathBuf;

use glam::Vec3A;
use motor_math::Motor;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThrusterConfigDefinition {
    #[serde(flatten)]
    pub thruster_config_type: ThrusterConfigTypeDefinition,

    pub center_of_mass: Vec3A,
    pub thruster_amperage_budget: f32,
    pub thruster_jerk_limit: f32,
    pub thruster_data_path: PathBuf,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ThrusterConfigTypeDefinition {
    X3d {
        seed_thruster: Motor,
    },
    BlueRov {
        vertical_seed_thruster: Motor,
        lateral_seed_thruster: Motor,
    },
    Custom,
}

pub type ThrusterDefinition = toml::Value;
