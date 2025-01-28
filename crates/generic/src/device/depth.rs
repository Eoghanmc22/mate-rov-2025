use crate::units::{Mbar, Meters};

pub trait DepthSensor {
    fn read_depth(&self) -> anyhow::Result<Meters>;
    fn read_altitude(&self) -> anyhow::Result<Meters>;
    fn read_pressure(&self) -> anyhow::Result<Mbar>;
}
