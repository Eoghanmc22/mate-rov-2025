use std::path::Path;

use anyhow::Context;
use serde::Deserialize;
use tracing::instrument;

use crate::{Direction, FloatType, Number};

pub struct MotorData {
    force_index: RecordIndex,
    current_index: RecordIndex,
}

impl MotorData {
    #[instrument(level = "trace", skip(self), ret)]
    pub fn lookup_by_force<D: Number>(
        &self,
        force: D,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let nearest_records = self.force_index.lookup_nearest(force.re());

        Self::interpolate(
            nearest_records.0,
            nearest_records.1,
            force,
            nearest_records.0.force,
            nearest_records.1.force,
            interpolation,
            extrapolate,
        )
    }

    #[instrument(level = "trace", skip(self), ret)]
    pub fn lookup_by_current<D: Number>(
        &self,
        signed_current: D,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let nearest_records = self.current_index.lookup_nearest(signed_current.re());

        Self::interpolate(
            nearest_records.0,
            nearest_records.1,
            signed_current,
            nearest_records.0.current.copysign(nearest_records.0.force),
            nearest_records.1.current.copysign(nearest_records.1.force),
            interpolation,
            extrapolate,
        )
    }

    fn interpolate<D: Number>(
        a: &MotorRecord<FloatType>,
        b: &MotorRecord<FloatType>,
        value: D,
        value_a: FloatType,
        value_b: FloatType,
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

impl From<Vec<MotorRecord<FloatType>>> for MotorData {
    fn from(value: Vec<MotorRecord<FloatType>>) -> Self {
        let mut force_index = value.clone();

        force_index.sort_by(|a, b| FloatType::total_cmp(&a.force, &b.force));
        force_index.dedup_by_key(|it| it.force);

        let mut current_index = value.clone();

        current_index.sort_by(|a, b| {
            FloatType::total_cmp(&a.current.copysign(a.force), &b.current.copysign(b.force))
        });
        current_index.dedup_by_key(|it| it.current.copysign(it.force));

        Self {
            force_index: RecordIndex::new(force_index, |it| it.force),
            current_index: RecordIndex::new(current_index, |it| it.current.copysign(it.force)),
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

fn lerp<D: Number>(a: FloatType, b: FloatType, alpha: D) -> D {
    (D::one() - alpha) * a + alpha * b
}

pub fn read_motor_data_from_path<P: AsRef<Path>>(path: P) -> anyhow::Result<MotorData> {
    let csv = csv::Reader::from_path(path).context("Read data")?;

    let mut data = Vec::default();
    for result in csv.into_deserialize() {
        let record: MotorRecord<FloatType> = result.context("Parse motor record")?;
        data.push(record);
    }

    Ok(data.into())
}

pub fn read_motor_data_from_string(data: &str) -> anyhow::Result<MotorData> {
    let csv = csv::Reader::from_reader(data.as_bytes());

    let mut data = Vec::default();
    for result in csv.into_deserialize() {
        let record: MotorRecord<FloatType> = result.context("Parse motor record")?;
        data.push(record);
    }

    Ok(data.into())
}

struct RecordIndex {
    data: Vec<MotorRecord<FloatType>>,
    supplier: Box<dyn Fn(&MotorRecord<FloatType>) -> FloatType + Send + Sync + 'static>,
}

impl RecordIndex {
    pub fn new(
        data: Vec<MotorRecord<FloatType>>,
        supplier: impl Fn(&MotorRecord<FloatType>) -> FloatType + Send + Sync + 'static,
    ) -> Self {
        Self {
            data,
            supplier: Box::new(supplier),
        }
    }

    pub fn lookup_nearest(
        &self,
        val: FloatType,
    ) -> (&MotorRecord<FloatType>, &MotorRecord<FloatType>) {
        let partition_point = self.data.partition_point(|x| (self.supplier)(x) < val);

        let idx_b = partition_point.max(1).min(self.data.len() - 1);
        let idx_a = idx_b - 1;

        let a = &self.data[idx_a];
        let b = &self.data[idx_b];

        (a, b)
    }
}
