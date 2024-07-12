use ahash::HashMap;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServoDefinition {
    pub name: String,

    #[serde(flatten)]
    pub interface: HashMap<String, toml::Value>,
}
