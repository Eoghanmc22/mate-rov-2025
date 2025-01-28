pub mod accelerometer;
pub mod adc;
pub mod depth;
pub mod gyroscope;
pub mod leak;
pub mod magnetometer;
pub mod motor;
pub mod temperature;

pub trait Multichannel<'a, Channel: 'a> {
    fn channels(&self) -> anyhow::Result<usize>;
    fn get_channel(&'a mut self, channel: usize) -> anyhow::Result<Channel>;
}
