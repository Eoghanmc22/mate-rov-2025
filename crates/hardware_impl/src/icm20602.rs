use generic::{
    Hardware,
    device::{accelerometer::Accelerometer, gyroscope::Gyroscope, temperature::TemperatureSensor},
    units::{Celsius, Dps, GForce, TypedVec3A},
};
use std::{thread, time::Duration};
use tracing::{debug, info, instrument};

use anyhow::{Context, bail};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

pub struct Icm20602 {
    spi: Spi,
    last_frame: Option<Icm20602Frame>,
    initialized: bool,
}

#[derive(Debug, Clone)]
pub struct Icm20602Frame {
    pub gyro: TypedVec3A<Dps>,
    pub accel: TypedVec3A<GForce>,
    pub temp: Celsius,
}

impl Icm20602 {
    pub const SPI_BUS: Bus = Bus::Spi1;
    pub const SPI_SELECT: SlaveSelect = SlaveSelect::Ss2;
    pub const SPI_CLOCK: u32 = 10_000_000;

    #[instrument(level = "debug")]
    pub fn new(bus: Bus, slave_select: SlaveSelect, clock_speed: u32) -> anyhow::Result<Self> {
        info!("Setting up ICM20602 (Gyro and Accelerometer)");

        let spi = Spi::new(bus, slave_select, clock_speed, Mode::Mode0).context("Open spi")?;

        Ok(Self {
            spi,
            last_frame: None,
            initialized: false,
        })
    }

    #[instrument(level = "trace", skip(self), ret)]
    pub fn read_frame(&mut self) -> anyhow::Result<Icm20602Frame> {
        let raw = self.read_raw_frame().context("Read raw frame")?;

        // The first byte is junk
        let raw = &raw[1..];

        let raw_accel_native_x = ((raw[0] as u16) << 8) | raw[1] as u16;
        let raw_accel_native_y = ((raw[2] as u16) << 8) | raw[3] as u16;
        let raw_accel_native_z = ((raw[4] as u16) << 8) | raw[5] as u16;

        let raw_temperature = ((raw[6] as u16) << 8) | raw[7] as u16;

        let raw_gyro_native_x = ((raw[8] as u16) << 8) | raw[9] as u16;
        let raw_gyro_native_y = ((raw[10] as u16) << 8) | raw[11] as u16;
        let raw_gyro_native_z = ((raw[12] as u16) << 8) | raw[13] as u16;

        let accel_native_x = raw_accel_native_x as i16 as f32 / 4096.0;
        let accel_native_y = raw_accel_native_y as i16 as f32 / 4096.0;
        let accel_native_z = raw_accel_native_z as i16 as f32 / 4096.0;

        let temperature = raw_temperature as i16 as f32 / 326.8 + 25.0;

        let gyro_native_x = raw_gyro_native_x as i16 as f32 / 16.4;
        let gyro_native_y = raw_gyro_native_y as i16 as f32 / 16.4;
        let gyro_native_z = raw_gyro_native_z as i16 as f32 / 16.4;

        let accel_x = -accel_native_y;
        let accel_y = -accel_native_x;
        let accel_z = -accel_native_z;

        let gyro_x = -gyro_native_y;
        let gyro_y = -gyro_native_x;
        let gyro_z = -gyro_native_z;

        let frame = Icm20602Frame {
            gyro: TypedVec3A::<Dps>::new_xyz(gyro_x, gyro_y, gyro_z),
            accel: TypedVec3A::<GForce>::new_xyz(accel_x, accel_y, accel_z),
            temp: Celsius(temperature),
        };

        self.last_frame = Some(frame.clone());

        Ok(frame)
    }
}

// Implementation based on https://github.com/bluerobotics/icm20602-python
impl Icm20602 {
    const REG_I2C_IF: u8 = 0x70;
    const REG_CONFIG: u8 = 0x1A;
    const REG_GYRO_CONFIG: u8 = 0x1B;
    const REG_ACCEL_CONFIG: u8 = 0x1C;
    const REG_ACCEL_CONFIG_2: u8 = 0x1D;
    const REG_ACCEL_INTEL_CTRL: u8 = 0x69;
    const REG_PWR_MGMT_1: u8 = 0x6B;
    const REG_WHO_AM_I: u8 = 0x75;
    const REG_ACCEL_XOUT_H: u8 = 0x3B;

    const READ: u8 = 0x80;

    fn read_raw_frame(&mut self) -> anyhow::Result<[u8; 15]> {
        assert!(self.initialized);

        let mut output = [0; 15];
        let mut input = [0; 15];

        output[0] = Self::REG_ACCEL_XOUT_H | Self::READ;

        self.spi
            .transfer(&mut input, &output)
            .context("Begin read imu frame")?;

        Ok(input)
    }
}

impl Hardware for Icm20602 {
    fn init(&mut self) -> anyhow::Result<()> {
        debug!("Initializing ICM20602 (gyro + accelerometer)");

        let mut id = [0, 0];
        self.spi
            .transfer(&mut id, &[Self::REG_WHO_AM_I | Self::READ, 0])
            .context("Request id")?;
        assert_eq!(id[1], 0x12);

        self.spi
            .write(&[Self::REG_I2C_IF, 0x40])
            .context("Disable i2c")?;

        // 1Hz sample rate
        self.spi
            .write(&[Self::REG_CONFIG, 0x1])
            .context("Setup lowpass filter")?;

        // 2000 deg range, lowpass filter
        self.spi
            .write(&[Self::REG_GYRO_CONFIG, 0b11 << 3])
            .context("Setup gyro")?;

        // 8g range
        self.spi
            .write(&[Self::REG_ACCEL_CONFIG, 0b10 << 3])
            .context("Setup accel")?;

        // lowpass filter
        self.spi
            .write(&[Self::REG_ACCEL_CONFIG_2, 0x0])
            .context("Setup accel")?;

        // Disable output limit
        self.spi
            .write(&[Self::REG_ACCEL_INTEL_CTRL, 0x2])
            .context("Setup accel")?;

        // Exit sleep mode
        self.spi
            .write(&[Self::REG_PWR_MGMT_1, 0x1])
            .context("Exit sleep")?;

        // Delay to allow sensors to start up and stabilize
        thread::sleep(Duration::from_millis(100));

        debug!("Initializing ICM20602 complete");

        self.initialized = true;

        Ok(())
    }

    fn poll(&mut self) -> anyhow::Result<()> {
        self.read_frame().context("Read frame")?;

        Ok(())
    }

    fn fastest_polling_interval(&self) -> anyhow::Result<Option<Duration>> {
        Ok(Some(Duration::from_secs_f32(1.0 / 1000.0)))
    }

    fn suggested_polling_interval(&self) -> anyhow::Result<Duration> {
        Ok(Duration::from_secs_f32(1.0 / 1000.0))
    }
}

impl Gyroscope for Icm20602 {
    fn read_gyroscope(&self) -> anyhow::Result<TypedVec3A<Dps>> {
        if let Some(last_frame) = &self.last_frame {
            return Ok(last_frame.gyro.clone());
        }

        bail!("Icm20602 Read before poll")
    }
}

impl Accelerometer for Icm20602 {
    fn read_acceleration(&self) -> anyhow::Result<TypedVec3A<GForce>> {
        if let Some(last_frame) = &self.last_frame {
            return Ok(last_frame.accel.clone());
        }

        bail!("Icm20602 Read before poll")
    }
}

impl TemperatureSensor for Icm20602 {
    fn read_temperature(&self) -> anyhow::Result<Celsius> {
        if let Some(last_frame) = &self.last_frame {
            return Ok(last_frame.temp.clone());
        }

        bail!("Icm20602 Read before poll")
    }
}
