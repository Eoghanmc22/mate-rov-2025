use std::path::Path;

use anyhow::Context;
use itertools::Itertools;
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
    pub fn binary_search_by_force<D: Number>(
        &self,
        force: D,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let nearest_records = self.force_index.binary_search_nearest(force.re());

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

    #[instrument(level = "trace", skip(self), ret)]
    pub fn binary_search_by_current<D: Number>(
        &self,
        signed_current: D,
        interpolation: Interpolation,
        extrapolate: bool,
    ) -> MotorRecord<D> {
        let nearest_records = self
            .current_index
            .binary_search_nearest(signed_current.re());

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
                    current: record.current.into(),
                    force: record.force.into(),

                    #[cfg(not(feature = "no_motor_control_data"))]
                    pwm: record.pwm.into(),
                    #[cfg(not(feature = "no_motor_control_data"))]
                    rpm: record.rpm.into(),
                    #[cfg(not(feature = "no_motor_control_data"))]
                    voltage: record.voltage.into(),
                    #[cfg(not(feature = "no_motor_control_data"))]
                    power: record.power.into(),
                    #[cfg(not(feature = "no_motor_control_data"))]
                    efficiency: record.efficiency.into(),
                }
            }
        };

        match interpolation {
            Interpolation::LerpDirection(direction) | Interpolation::Direction(direction) => {
                if let Direction::CounterClockwise = direction {
                    MotorRecord {
                        #[cfg(not(feature = "no_motor_control_data"))]
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
    /// Return the linear interpolation between the two data entries closest to the the requested data point
    /// and modifies the pwm field to match the direction of the propeller
    LerpDirection(Direction),
    /// Return the raw data entry closest to the the requested data point
    /// Only modifies the pwm field to match the direction of the propeller
    Direction(Direction),
    /// Return the linear interpolation between the two data entries closest to the the requested data point
    #[default]
    Lerp,
    /// Return the raw data entry closest to the the requested data point
    /// Make no modifications to the data
    OriginalData,
}

#[derive(Deserialize, Debug, Clone, Copy, Default, PartialEq)]
pub struct MotorRecord<D> {
    pub current: D,
    pub force: D,

    #[cfg(not(feature = "no_motor_control_data"))]
    pub pwm: D,
    #[cfg(not(feature = "no_motor_control_data"))]
    pub rpm: D,
    #[cfg(not(feature = "no_motor_control_data"))]
    pub voltage: D,
    #[cfg(not(feature = "no_motor_control_data"))]
    pub power: D,
    #[cfg(not(feature = "no_motor_control_data"))]
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
            current: lerp(self.current.re(), other.current.re(), alpha),
            force: lerp(self.force.re(), other.force.re(), alpha),

            #[cfg(not(feature = "no_motor_control_data"))]
            pwm: lerp(self.pwm.re(), other.pwm.re(), alpha),
            #[cfg(not(feature = "no_motor_control_data"))]
            rpm: lerp(self.rpm.re(), other.rpm.re(), alpha),
            #[cfg(not(feature = "no_motor_control_data"))]
            voltage: lerp(self.voltage.re(), other.voltage.re(), alpha),
            #[cfg(not(feature = "no_motor_control_data"))]
            power: lerp(self.power.re(), other.power.re(), alpha),
            #[cfg(not(feature = "no_motor_control_data"))]
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
    lookup_table: Box<
        [(
            MotorRecord<FloatType>,
            Option<MotorRecord<FloatType>>,
            MotorRecord<FloatType>,
        )],
    >,
    float_compression: FloatCompression,
    supplier: Box<dyn Fn(&MotorRecord<FloatType>) -> FloatType + Send + Sync + 'static>,
}

#[derive(Debug)]
struct FloatCompression {
    min: FloatType,
    max: FloatType,
    steps: isize,
}

impl FloatCompression {
    pub fn compress(&self, float: FloatType) -> isize {
        // ((float - self.min) / (self.max - self.min) * (self.steps - 1) as FloatType) as isize
        ((float - self.min) / (self.max - self.min) * (self.steps - 1) as FloatType).floor()
            as isize
    }
    pub fn decompress(&self, idx: isize) -> (FloatType, FloatType) {
        (
            (idx as FloatType + 0.0) / (self.steps - 1) as FloatType * (self.max - self.min)
                + self.min,
            (idx as FloatType + 1.0) / (self.steps - 1) as FloatType * (self.max - self.min)
                + self.min,
        )
    }
}

impl RecordIndex {
    pub fn new(
        data: Vec<MotorRecord<FloatType>>,
        supplier: impl Fn(&MotorRecord<FloatType>) -> FloatType + Send + Sync + 'static,
    ) -> Self {
        assert!(
            data.len() >= 2,
            "The data in a record index needs to have at least two elements"
        );

        let min = data.first().map(&supplier).unwrap();
        let max = data.last().map(&supplier).unwrap();

        let min_step_size = data
            .iter()
            .map(&supplier)
            .tuple_windows()
            .map(|(a, b)| b - a)
            .min_by(FloatType::total_cmp)
            .unwrap();
        let steps = ((max - min) / min_step_size).ceil() as usize + 1;

        let compression = FloatCompression {
            min,
            max,
            steps: steps as isize,
        };

        let mut lookup_table = Vec::with_capacity(steps);
        for step in 0..steps {
            let (low_value, high_value) = compression.decompress(step as isize);
            let (low, mid1) = binary_search_nearest_internal(low_value, &data, &supplier);
            let (mid2, high) = binary_search_nearest_internal(high_value, &data, &supplier);

            let mid = if mid1 == mid2 {
                Some(*mid1)
            } else {
                assert!((supplier)(mid1) > (supplier)(mid2), "Data point skipped");
                None
            };

            lookup_table.push((*low, mid, *high));
        }

        Self {
            data,
            lookup_table: lookup_table.into_boxed_slice(),
            float_compression: compression,
            supplier: Box::new(supplier),
        }
    }

    pub fn binary_search_nearest(
        &self,
        val: FloatType,
    ) -> (&MotorRecord<FloatType>, &MotorRecord<FloatType>) {
        binary_search_nearest_internal(val, &self.data, &self.supplier)
    }

    pub fn lookup_nearest(
        &self,
        val: FloatType,
    ) -> (&MotorRecord<FloatType>, &MotorRecord<FloatType>) {
        let idx =
            (self.float_compression.compress(val).max(0) as usize).min(self.lookup_table.len() - 1);
        let (low, mid, high) = &self.lookup_table[idx];

        if let Some(mid) = mid {
            if ((self.supplier)(low)..=(self.supplier)(mid)).contains(&val) {
                (low, mid)
            } else {
                (mid, high)
            }
        } else {
            (low, high)
        }
    }
}

fn binary_search_nearest_internal(
    val: FloatType,
    data: &[MotorRecord<FloatType>],
    supplier: impl Fn(&MotorRecord<FloatType>) -> FloatType + Send + Sync,
) -> (&MotorRecord<FloatType>, &MotorRecord<FloatType>) {
    let partition_point = data.partition_point(|x| (supplier)(x) < val);

    let idx_b = partition_point.max(1).min(data.len() - 1);
    let idx_a = idx_b - 1;

    let a = &data[idx_a];
    let b = &data[idx_b];

    (a, b)
}

#[cfg(test)]
mod tests {
    use crate::FloatType;

    #[test]
    fn check_force_lookup_table() {
        let motor_data =
            super::read_motor_data_from_path("../robot/motor_data.csv").expect("Read motor data");
        let float_compression = &motor_data.force_index.float_compression;
        let epsilon = 0.0001;

        let steps = 2000;
        for step in 0..steps {
            let point = (float_compression.max - float_compression.min) / steps as FloatType
                * step as FloatType
                + float_compression.min;

            let lookup = motor_data.lookup_by_force(point, super::Interpolation::Lerp, false);
            let binary_search =
                motor_data.binary_search_by_force(point, super::Interpolation::Lerp, false);

            dbg!(&point);
            dbg!(&lookup.force);
            dbg!(&lookup.current);
            dbg!(&binary_search.force);
            dbg!(&binary_search.current);

            assert!((lookup.force - point).abs() < epsilon);
            assert!((binary_search.force - point).abs() < epsilon);
            assert!((binary_search.current - lookup.current).abs() < epsilon);
            assert_eq!(lookup, binary_search);

            println!();
        }
    }

    #[test]
    fn check_current_lookup_table() {
        let motor_data =
            super::read_motor_data_from_path("../robot/motor_data.csv").expect("Read motor data");
        let float_compression = &motor_data.current_index.float_compression;
        let epsilon = 0.0001;

        let steps = 2000;
        for step in 0..steps {
            let point = (float_compression.max - float_compression.min) / steps as FloatType
                * step as FloatType
                + float_compression.min;

            let lookup = motor_data.lookup_by_current(point, super::Interpolation::Lerp, false);
            let binary_search =
                motor_data.binary_search_by_current(point, super::Interpolation::Lerp, false);

            dbg!(&point);
            dbg!(&lookup.force);
            dbg!(&lookup.current);
            dbg!(&binary_search.force);
            dbg!(&binary_search.current);

            assert!((lookup.current.copysign(lookup.force) - point).abs() < epsilon);
            assert!((binary_search.current.copysign(binary_search.force) - point).abs() < epsilon);
            assert!((binary_search.force - lookup.force).abs() < epsilon);
            assert_eq!(lookup, binary_search);

            println!();
        }
    }
}
