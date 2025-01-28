use crate::units::Celsius;

pub trait TemperatureSensor {
    fn read_temperature(&self) -> anyhow::Result<Celsius>;
}
