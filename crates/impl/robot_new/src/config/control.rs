use common::components::PidConfig;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlSystemDefinition {
    pub depth_hold: PidConfig,
    pub stabilize: StabilizeDefinition,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StabilizeDefinition {
    pub pitch: PidConfig,
    pub yaw: PidConfig,
    pub roll: PidConfig,
}
