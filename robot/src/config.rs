use ahash::{HashMap, HashSet};
use bevy::{ecs::system::Resource, transform::components::Transform};
use common::components::{MotorContributionMode, MotorSignalType, PidConfig};
use glam::{vec3, vec3a, EulerRot, Quat, Vec3A};
use motor_math::{
    blue_rov::BlueRovMotorId, blue_rov_heavy::HeavyMotorId, glam::ThrusterGlam, x3d::X3dMotorId,
    ErasedMotorId, MotorConfig,
};
use nalgebra::vector;
use serde::{Deserialize, Serialize};

use crate::plugins::actuators::{hardware::motor_id_map::LocalMotorId, stabilize::PidAxis};

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
    pub name: String,
    pub port: u16,

    pub motor_config: MotorConfigDefinition,
    #[serde(default)]
    pub servo_config: ServoConfigDefinition,

    pub motor_amperage_budget: f32,
    pub jerk_limit: f32,
    pub center_of_mass: Vec3A,

    pub imu_offset: ConfigRotation,

    #[serde(default)]
    pub cameras: HashMap<String, CameraDefinition>,

    pub pid_configs: HashMap<PidAxis, PidConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MotorConfigDefinition {
    X3d(X3dDefinition),
    BlueRov(BlueRovDefinition),
    Heavy(HeavyDefinition),
    Custom(CustomDefinition),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct X3dDefinition {
    pub seed_motor: ThrusterGlam,

    pub motors: HashMap<X3dMotorId, LocalMotorId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlueRovDefinition {
    pub vertical_seed_motor: ThrusterGlam,
    pub lateral_seed_motor: ThrusterGlam,

    pub motors: HashMap<BlueRovMotorId, LocalMotorId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeavyDefinition {
    pub vertical_seed_motor: ThrusterGlam,
    pub lateral_seed_motor: ThrusterGlam,

    pub motors: HashMap<HeavyMotorId, LocalMotorId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomDefinition {
    pub motors: HashMap<String, CustomThruster>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomThruster {
    pub channel: LocalMotorId,
    #[serde(flatten)]
    pub motor: ThrusterGlam,
}

impl X3dDefinition {
    fn to_motor_config(
        &self,
        center_mass: Vec3A,
    ) -> MotorConfig<X3dMotorId, motor_math::FloatType> {
        MotorConfig::<X3dMotorId, motor_math::FloatType>::new(
            self.seed_motor.into(),
            vector![center_mass.x as _, center_mass.y as _, center_mass.z as _],
        )
    }
}

impl BlueRovDefinition {
    fn to_motor_config(
        &self,
        center_mass: Vec3A,
    ) -> MotorConfig<BlueRovMotorId, motor_math::FloatType> {
        MotorConfig::<BlueRovMotorId, motor_math::FloatType>::new(
            self.lateral_seed_motor.into(),
            self.vertical_seed_motor.into(),
            vector![center_mass.x as _, center_mass.y as _, center_mass.z as _],
        )
    }
}

impl HeavyDefinition {
    fn to_motor_config(
        &self,
        center_mass: Vec3A,
    ) -> MotorConfig<HeavyMotorId, motor_math::FloatType> {
        MotorConfig::<HeavyMotorId, motor_math::FloatType>::new(
            self.lateral_seed_motor.into(),
            self.vertical_seed_motor.into(),
            vector![center_mass.x as _, center_mass.y as _, center_mass.z as _],
        )
    }
}

impl CustomDefinition {
    fn to_motor_config(&self, center_mass: Vec3A) -> MotorConfig<String, motor_math::FloatType> {
        MotorConfig::<String, motor_math::FloatType>::new_raw(
            self.motors
                .iter()
                .map(|(id, motor)| (id.to_owned(), motor.motor.into())),
            vector![center_mass.x as _, center_mass.y as _, center_mass.z as _],
        )
    }
}

impl MotorConfigDefinition {
    // TODO(low): Rename and make less bad
    pub fn flatten(
        &self,
        center_mass: Vec3A,
    ) -> (
        impl Iterator<Item = (ErasedMotorId, ThrusterGlam, LocalMotorId)>,
        MotorConfig<ErasedMotorId, motor_math::FloatType>,
    ) {
        let motors: Vec<_>;

        let config = match self {
            MotorConfigDefinition::X3d(x3d) => {
                let config: MotorConfig<_, motor_math::FloatType> =
                    x3d.to_motor_config(center_mass);

                motors = config
                    .motors()
                    .map(|(id, motor)| {
                        (
                            (*id).into(),
                            *motor,
                            x3d.motors
                                .get(id)
                                .copied()
                                .expect("Incomplete motor definition"),
                        )
                    })
                    .collect();

                config.erase()
            }
            MotorConfigDefinition::Heavy(heavy) => {
                let config: MotorConfig<_, motor_math::FloatType> =
                    heavy.to_motor_config(center_mass);

                motors = config
                    .motors()
                    .map(|(id, motor)| {
                        (
                            (*id).into(),
                            *motor,
                            heavy
                                .motors
                                .get(id)
                                .copied()
                                .expect("Incomplete motor definition"),
                        )
                    })
                    .collect();

                config.erase()
            }
            MotorConfigDefinition::BlueRov(blue_rov) => {
                let config: MotorConfig<_, motor_math::FloatType> =
                    blue_rov.to_motor_config(center_mass);

                motors = config
                    .motors()
                    .map(|(id, motor)| {
                        (
                            (*id).into(),
                            *motor,
                            blue_rov
                                .motors
                                .get(id)
                                .copied()
                                .expect("Incomplete motor definition"),
                        )
                    })
                    .collect();

                config.erase()
            }
            MotorConfigDefinition::Custom(custom) => {
                let config: MotorConfig<_, motor_math::FloatType> =
                    custom.to_motor_config(center_mass);

                motors = config
                    .motors()
                    .enumerate()
                    .map(|(idx, (id, motor))| {
                        (
                            idx as u8,
                            *motor,
                            custom
                                .motors
                                .get(id)
                                .map(|it| it.channel)
                                .expect("Incomplete motor definition"),
                        )
                    })
                    .collect();

                MotorConfig::new_raw(
                    config
                        .motors()
                        .enumerate()
                        .map(|(idx, (_, motor))| (idx as _, *motor)),
                    vector![center_mass.x as _, center_mass.y as _, center_mass.z as _],
                )
            }
        };

        (
            motors
                .into_iter()
                .map(|(idx, motor, id)| (idx, motor.into(), id)),
            config,
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ServoConfigDefinition {
    pub servos: HashMap<String, Servo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Servo {
    pub channel: LocalMotorId,
    pub signal_type: MotorSignalType,
    // pub cameras: HashSet<String>,
    pub camera: Option<String>,
    pub constraints: Option<ServoConstraints>,
    pub control_mode: Option<MotorContributionMode>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServoConstraints {
    pub min: f32,
    pub max: f32,
}

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct CameraDefinition {
    pub name: String,
    pub transform: ConfigTransform,
}

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct ConfigTransform {
    position: ConfigPosition,
    rotation: ConfigRotation,
}

impl ConfigTransform {
    pub fn flatten(&self) -> Transform {
        Transform::from_translation(self.position.flatten().into())
            .with_rotation(self.rotation.flatten())
    }
}

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct ConfigPosition {
    x: f32,
    y: f32,
    z: f32,
}

impl ConfigPosition {
    pub fn flatten(&self) -> Vec3A {
        let &ConfigPosition { x, y, z } = self;

        vec3a(x, y, z)
    }
}

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct ConfigRotation {
    yaw: f32,
    pitch: f32,
    roll: f32,
}

impl ConfigRotation {
    pub fn flatten(&self) -> Quat {
        let &ConfigRotation { yaw, pitch, roll } = self;

        Quat::from_euler(
            EulerRot::ZXY,
            yaw.to_radians(),
            pitch.to_radians(),
            roll.to_radians(),
        )
    }
}
