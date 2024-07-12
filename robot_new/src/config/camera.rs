use bevy::transform::components::Transform;
use glam::{vec3, EulerRot, Quat};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraDefinition {
    pub name: String,
    #[serde(flatten)]
    pub camera_type: CameraTypeDefinition,
    pub transform: Option<ConfigTransform>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CameraTypeDefinition {
    H264,
    // MJPEG,
    // GSTREAMER { tx: String, rx: String},
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(into = "ConfigTransformImpl")]
#[serde(from = "ConfigTransformImpl")]
pub struct ConfigTransform(pub Transform);

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ConfigTransformImpl {
    position: ConfigPosition,
    rotation: ConfigRotation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ConfigPosition {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ConfigRotation {
    yaw: f32,
    pitch: f32,
    roll: f32,
}

impl From<ConfigTransformImpl> for ConfigTransform {
    fn from(value: ConfigTransformImpl) -> ConfigTransform {
        let ConfigPosition { x, y, z } = value.position;
        let ConfigRotation { yaw, pitch, roll } = value.rotation;

        // TODO: What is this...
        ConfigTransform(
            Transform::from_translation(Quat::from_rotation_x(90f32.to_radians()) * vec3(x, -y, z))
                .with_rotation(Quat::from_euler(
                    EulerRot::default(),
                    yaw.to_radians(),
                    pitch.to_radians(),
                    roll.to_radians(),
                )),
        )
    }
}

impl From<ConfigTransform> for ConfigTransformImpl {
    fn from(value: ConfigTransform) -> Self {
        // FIXME: Actually impl this when im not tired lol
        todo!()
    }
}
