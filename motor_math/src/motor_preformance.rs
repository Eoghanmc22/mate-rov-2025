use std::path::Path;

use anyhow::Context;
use serde::Deserialize;
use tracing::instrument;

use crate::{Direction, Number};

pub struct MotorData {
    force_index: Vec<MotorRecord<f32>>,
    current_index: Vec<MotorRecord<f32>>,
}

impl MotorData {
    #[instrument(level = "trace", skip(self), ret)]
    pub fn lookup_by_force<D: Number>(
        &self,
        force: D,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let partition_point = self.force_index.partition_point(|x| x.force < force.re());

        let idx_b = partition_point.max(1).min(self.force_index.len() - 1);
        let idx_a = idx_b - 1;

        let a = &self.force_index[idx_a];
        let b = &self.force_index[idx_b];

        Self::interpolate(a, b, force, a.force, b.force, interpolation, extrapolate)
    }

    #[instrument(level = "trace", skip(self), ret)]
    pub fn lookup_by_current<D: Number>(
        &self,
        signed_current: D,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let partition_point = self
            .current_index
            .partition_point(|x| x.current.copysign(x.force) < signed_current.re());

        let idx_b = partition_point.max(1).min(self.current_index.len() - 1);
        let idx_a = idx_b - 1;

        let a = &self.current_index[idx_a];
        let b = &self.current_index[idx_b];

        Self::interpolate(
            a,
            b,
            signed_current,
            a.current.copysign(a.force),
            b.current.copysign(b.force),
            interpolation,
            extrapolate,
        )
    }

    fn interpolate<D: Number>(
        a: &MotorRecord<f32>,
        b: &MotorRecord<f32>,
        value: D,
        value_a: f32,
        value_b: f32,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let record = match interpolation {
            Interpolation::LerpDirection(_) | Interpolation::Lerp => {
                let alpha = (value - value_a) / (value_b - value_a);
                a.lerp(b, alpha, extrapolate)
            }
            Interpolation::Direction(_) | Interpolation::OriginalData => {
                let dist_a = (value_a - value.re()).abs();
                let dist_b = (value_b - value.re()).abs();

                let record = if dist_a <= dist_b { a } else { b };

                MotorRecord {
                    pwm: record.pwm.into(),
                    rpm: record.rpm.into(),
                    current: record.current.into(),
                    voltage: record.voltage.into(),
                    power: record.power.into(),
                    force: record.force.into(),
                    efficiency: record.efficiency.into(),
                }
            }
        };

        match interpolation {
            Interpolation::LerpDirection(direction) | Interpolation::Direction(direction) => {
                if let Direction::CounterClockwise = direction {
                    MotorRecord {
                        pwm: D::from(3000.0) - record.pwm,
                        ..record
                    }
                } else {
                    record
                }
            }
            Interpolation::Lerp | Interpolation::OriginalData => record,
        }
    }
}

impl From<Vec<MotorRecord<f32>>> for MotorData {
    fn from(value: Vec<MotorRecord<f32>>) -> Self {
        let mut force_index = value.clone();

        force_index.sort_by(|a, b| f32::total_cmp(&a.force, &b.force));
        force_index.dedup_by_key(|it| it.force);

        let mut current_index = value.clone();

        current_index.sort_by(|a, b| {
            f32::total_cmp(&a.current.copysign(a.force), &b.current.copysign(b.force))
        });
        current_index.dedup_by_key(|it| it.current.copysign(it.force));

        Self {
            force_index,
            current_index,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub enum Interpolation {
    /// Return the linear interpolation betwwn the two data entries closest to the the requested data point
    /// and modifies the pwm field to match the direction of the propeller
    LerpDirection(Direction),
    /// Return the raw data entry closest to the the requested data point
    /// Only modifies the pwm field to match the direction of the propeller
    Direction(Direction),
    /// Return the linear interpolation betwwn the two data entries closest to the the requested data point
    #[default]
    Lerp,
    /// Return the raw data entry closest to the the requested data point
    /// Make no modifications to the data
    OriginalData,
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct MotorRecord<D> {
    pub pwm: D,
    pub rpm: D,
    pub current: D,
    pub voltage: D,
    pub power: D,
    pub force: D,
    pub efficiency: D,
}

impl<D1: Number> MotorRecord<D1> {
    // This goofy generics stuff should allow the motor data tables to be in f32 and alpha to be a dual num
    pub fn lerp<D2: Number>(
        &self,
        other: &Self,
        mut alpha: D2,
        extrapolate: bool,
    ) -> MotorRecord<D2> {
        if !extrapolate {
            alpha = alpha.clamp(D2::zero(), D2::one());
        }

        MotorRecord {
            pwm: lerp(self.pwm.re(), other.pwm.re(), alpha),
            rpm: lerp(self.rpm.re(), other.rpm.re(), alpha),
            current: lerp(self.current.re(), other.current.re(), alpha),
            voltage: lerp(self.voltage.re(), other.voltage.re(), alpha),
            power: lerp(self.power.re(), other.power.re(), alpha),
            force: lerp(self.force.re(), other.force.re(), alpha),
            efficiency: lerp(self.efficiency.re(), other.efficiency.re(), alpha),
        }
    }
}

fn lerp<D: Number>(a: f32, b: f32, alpha: D) -> D {
    (D::one() - alpha) * a + alpha * b
}

pub fn read_motor_data<P: AsRef<Path>>(path: P) -> anyhow::Result<MotorData> {
    let csv = csv::Reader::from_path(path).context("Read data")?;

    let mut data = Vec::default();
    for result in csv.into_deserialize() {
        let record: MotorRecord<f32> = result.context("Parse motor record")?;
        data.push(record);
    }

    Ok(data.into())
}
