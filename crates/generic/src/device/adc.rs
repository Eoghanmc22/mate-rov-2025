use crate::units::Volts;

pub trait AnalogueToDigitalConverter {
    fn read(&self) -> anyhow::Result<Volts>;
}
