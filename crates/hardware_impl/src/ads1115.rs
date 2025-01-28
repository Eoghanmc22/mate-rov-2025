use std::{thread, time::Duration};

use generic::{
    Hardware,
    device::{Multichannel, adc::AnalogueToDigitalConverter},
    units::Volts,
};
use rppal::i2c::I2c;
use tracing::{info, instrument};

use anyhow::{Context, Ok, bail};

pub struct Ads1115 {
    i2c: I2c,
    cached_valued: Option<[Volts; 4]>,
}

impl Ads1115 {
    pub const I2C_BUS: u8 = 1;
    pub const I2C_ADDRESS: u8 = 0x48;

    #[instrument(level = "debug")]
    pub fn new(bus: u8, address: u8) -> anyhow::Result<Self> {
        info!("Setting up ADS1115 (ADC)");

        let mut i2c = I2c::with_bus(bus).context("Open i2c")?;

        i2c.set_slave_address(address as u16)
            .context("Set address for ADS1115")?;

        Ok(Self {
            i2c,
            cached_valued: None,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Ads1115ChannelId {
    Ch0,
    Ch1,
    Ch2,
    Ch3,
}

impl Ads1115ChannelId {
    pub fn selector(&self) -> u16 {
        match self {
            Ads1115ChannelId::Ch0 => 0b100,
            Ads1115ChannelId::Ch1 => 0b101,
            Ads1115ChannelId::Ch2 => 0b110,
            Ads1115ChannelId::Ch3 => 0b111,
        }
    }

    pub fn idx(&self) -> usize {
        match self {
            Ads1115ChannelId::Ch0 => 0,
            Ads1115ChannelId::Ch1 => 1,
            Ads1115ChannelId::Ch2 => 2,
            Ads1115ChannelId::Ch3 => 3,
        }
    }
}

// Implementation based on https://github.com/bluerobotics/ads1115-python
impl Ads1115 {
    const POINTER_CONVERSION: u8 = 0x00;
    const POINTER_CONFIG: u8 = 0x01;

    #[instrument(level = "trace", skip(self), ret)]
    pub fn request_conversion(&mut self, channel: Ads1115ChannelId) -> anyhow::Result<()> {
        let config =
            (1 << 15) | (channel.selector() << 12) | (0b001 << 9) | (1 << 8) | (0b111 << 5);

        self.i2c
            .block_write(Self::POINTER_CONFIG, &config.to_be_bytes())
            .context("Begin ADC convert")?;

        Ok(())
    }

    #[instrument(level = "trace", skip(self), ret)]
    pub fn ready(&mut self) -> anyhow::Result<bool> {
        let mut buffer = [0u8; 2];

        self.i2c
            .block_read(Self::POINTER_CONFIG, &mut buffer)
            .context("Check ADC conversion status")?;

        let value = i16::from_be_bytes(buffer);

        Ok(value & (1 << 15) != 0)
    }

    #[instrument(level = "trace", skip(self), ret)]
    pub fn read(&mut self) -> anyhow::Result<Volts> {
        let mut buffer = [0u8; 2];

        self.i2c
            .block_read(Self::POINTER_CONVERSION, &mut buffer)
            .context("Check ADC conversion status")?;

        let value = u16::from_be_bytes(buffer);

        Ok(Volts(value as f32 / 0xffff as f32 * 2.0 * 4.096))
    }
}

impl Hardware for Ads1115 {
    fn init(&mut self) -> anyhow::Result<()> {
        // Nothing to init
        Ok(())
    }

    fn poll(&mut self) -> anyhow::Result<()> {
        use Ads1115ChannelId::*;

        let mut cache = [Volts(0.0); 4];
        for channel in [Ch0, Ch1, Ch2, Ch3] {
            self.request_conversion(channel)
                .context("Request conversion");
            thread::sleep(Duration::from_secs_f32(1.0 / 860.0));

            let mut iters = 0;
            while !self.ready().context("Check adc ready")? {
                if iters > 10 {
                    bail!("Hit max iters when reading ads1115");
                }

                thread::yield_now();

                iters += 1;
            }

            cache[channel.idx()] = self.read().context("Read adc")?;
        }

        self.cached_valued = Some(cache);

        Ok(())
    }

    fn fastest_polling_interval(&self) -> anyhow::Result<Option<Duration>> {
        Ok(None)
    }

    fn suggested_polling_interval(&self) -> anyhow::Result<Duration> {
        Ok(Duration::from_secs_f32(1.0 / 10.0))
    }
}

impl<'a> Multichannel<'a, Ads1115ChannelState> for Ads1115 {
    fn channels(&self) -> anyhow::Result<usize> {
        Ok(4)
    }

    fn get_channel(&'a mut self, channel: usize) -> anyhow::Result<Ads1115ChannelState> {
        let Some(cache) = &self.cached_valued else {
            bail!("Get channel called on Ads1115 before poll");
        };

        if channel >= cache.len() {
            bail!("Unknown channel: {channel}");
        }

        Ok(Ads1115ChannelState {
            last_reading: cache[channel],
        })
    }
}

pub struct Ads1115ChannelState {
    last_reading: Volts,
}

impl AnalogueToDigitalConverter for Ads1115ChannelState {
    fn read(&self) -> anyhow::Result<Volts> {
        Ok(self.last_reading)
    }
}
