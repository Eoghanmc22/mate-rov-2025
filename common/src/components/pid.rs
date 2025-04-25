use bevy::{
    ecs::component::Component,
    reflect::{prelude::ReflectDefault, Reflect, ReflectDeserialize, ReflectSerialize},
};
use serde::{Deserialize, Serialize};
use std::time::Duration;

use crate::adapters::serde::ReflectSerdeAdapter;

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub d_alpha: f32,

    pub i_zone: f32,
    pub max_integral: f32,
    pub max_output: f32,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct PidResult {
    pub error: f32,
    pub p: f32,
    pub i: f32,
    pub d: f32,

    pub correction: f32,
}

#[derive(Component, Clone, Copy, Debug, Serialize, Deserialize, Reflect, Default)]
#[reflect(Serialize, Deserialize, Debug, Default)]
pub struct PidController {
    last_error: Option<f32>,
    // last_derivative: Option<f32>,
    integral: f32,
}

impl PidController {
    pub fn new() -> Self {
        Self {
            last_error: None,
            // last_derivative: None,
            integral: 0.0,
        }
    }

    pub fn update(&mut self, error: f32, config: &PidConfig, interval: Duration) -> PidResult {
        let cfg = config;
        let interval = interval.as_secs_f32();

        self.integral += error * interval;
        self.integral = self.integral.clamp(-cfg.max_integral, cfg.max_integral);

        let proportional = error;
        let integral = self.integral;
        let derivative = if let Some(last_error) = self.last_error {
            let filtered_error = error * config.d_alpha + last_error * (1.0 - config.d_alpha);
            self.last_error = Some(filtered_error);

            (filtered_error - last_error) / interval
        } else {
            self.last_error = Some(error);
            0.0
        };

        // self.last_derivative = Some(derivative);
        self.last_error = Some(error);

        let p = cfg.kp * proportional;
        let i = cfg.ki * integral;
        let d = cfg.kd * derivative;

        let i = if error.abs() < config.i_zone {
            i
        } else {
            self.integral = 0.0;

            0.0
        };

        let correction = (p + i + d).clamp(-config.max_output, config.max_output);

        PidResult {
            error,
            p,
            i,
            d,
            correction,
        }
    }

    pub fn reset(&mut self) {
        *self = Default::default();
    }

    pub fn last_error(&self) -> f32 {
        self.last_error.unwrap_or_default()
    }

    pub fn integral(&self) -> f32 {
        self.integral
    }
}
