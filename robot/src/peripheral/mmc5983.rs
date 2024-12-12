use common::types::hw::MagneticFrame;
use common::types::units::Gauss;
use glam::{vec3a, Vec3A};
use std::{thread, time::Duration};
use tracing::{debug, info, instrument, trace};

use anyhow::Context;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

pub struct Mcc5983 {
    spi: Spi,
    offset: [f32; 3],
}

impl Mcc5983 {
    pub const SPI_BUS: Bus = Bus::Spi1;
    pub const SPI_SELECT: SlaveSelect = SlaveSelect::Ss1;
    pub const SPI_CLOCK: u32 = 10_000_000;

    #[instrument(level = "debug")]
    pub fn new(bus: Bus, slave_select: SlaveSelect, clock_speed: u32) -> anyhow::Result<Self> {
        info!("Setting up MCC5983 (Magnetometer)");

        let spi = Spi::new(bus, slave_select, clock_speed, Mode::Mode0).context("Open spi")?;

        let mut this = Self {
            spi,
            offset: [0.0; 3],
        };
        this.initialize().context("Initialize")?;

        Ok(this)
    }

    // TODO(high): Hard and soft iron calibration?

    #[instrument(level = "trace", skip(self), ret)]
    pub fn read_frame(&mut self) -> anyhow::Result<MagneticFrame> {
        let raw = self.read_raw_frame().context("Read raw frame")?;

        // The first byte is junk
        let raw = &raw[1..];

        let raw_mag_native_x =
            (raw[0] as u32) << 10 | (raw[1] as u32) << 2 | (raw[6] as u32 & 0xC0) >> 6;
        let raw_mag_native_y =
            (raw[2] as u32) << 10 | (raw[3] as u32) << 2 | (raw[6] as u32 & 0x30) >> 4;
        let raw_mag_native_z =
            (raw[4] as u32) << 10 | (raw[5] as u32) << 2 | (raw[6] as u32 & 0x0C) >> 2;

        let mag_native_x = (raw_mag_native_x as i32 - 131072) as f32 / 16384.0;
        let mag_native_y = (raw_mag_native_y as i32 - 131072) as f32 / 16384.0;
        let mag_native_z = (raw_mag_native_z as i32 - 131072) as f32 / 16384.0;

        let mag_x = mag_native_y - self.offset[1];
        let mag_y = mag_native_x - self.offset[0];
        let mag_z = mag_native_z - self.offset[2];

        let adjusted = MagneticCalibration::HARD_CODED.adjust(vec3a(mag_x, mag_y, mag_z));

        Ok(MagneticFrame {
            mag_x: Gauss(adjusted.x),
            mag_y: Gauss(adjusted.y),
            mag_z: Gauss(adjusted.z),
        })
    }
}

// Implementation based on https://github.com/bluerobotics/mmc5983-python/
impl Mcc5983 {
    const REG_XOUT_L: u8 = 0x00;
    const REG_STATUS: u8 = 0x08;
    const REG_CONTROL0: u8 = 0x09;
    const REG_CONTROL1: u8 = 0x0A;
    const REG_CONTROL2: u8 = 0x0B;
    const REG_WHO_AM_I: u8 = 0x2F;

    const READ: u8 = 0x80;

    fn initialize(&mut self) -> anyhow::Result<()> {
        debug!("Initializing MCC5982 (magnetometer)");

        // Software reset
        self.spi
            .write(&[Self::REG_CONTROL1, 0x80])
            .context("Software reset")?;
        thread::sleep(Duration::from_millis(15));

        // Read chip id
        let mut id = [0, 0];
        self.spi
            .transfer(&mut id, &[Self::REG_WHO_AM_I | Self::READ, 0])
            .context("Request id")?;
        assert_eq!(id[1], 0x30);

        // We are using the default bandwidth (100 Hz)
        // No need to set `REG_CONTROL1`

        self.calibrate_offset().context("calibrate")?;

        // Enable continous mode @ 100 Hz
        self.spi
            .write(&[Self::REG_CONTROL2, 0x0D])
            .context("Continous mode")?;

        debug!("Initializing MCC5982 complete");

        Ok(())
    }

    pub fn calibrate_offset(&mut self) -> anyhow::Result<()> {
        debug!("Calibrating MCC5982");

        self.offset = [0.0; 3];

        // SET
        self.spi
            .write(&[Self::REG_CONTROL0, 0x08])
            .context("Set mode")?;
        thread::sleep(Duration::from_millis(1));

        // Measure
        self.spi
            .write(&[Self::REG_CONTROL0, 0x01])
            .context("Measure")?;
        thread::sleep(Duration::from_millis(10));
        assert_eq!(
            self.read_reg(Self::REG_STATUS).context("Read status")? & 1,
            1
        );

        let set = self.read_frame().context("Read Set")?;
        trace!(?set, "Set calibration");

        // RESET
        self.spi
            .write(&[Self::REG_CONTROL0, 0x10])
            .context("Reset mode")?;
        thread::sleep(Duration::from_millis(1));

        // Measure
        self.spi
            .write(&[Self::REG_CONTROL0, 0x01])
            .context("Measure")?;
        thread::sleep(Duration::from_millis(10));
        assert_eq!(
            self.read_reg(Self::REG_STATUS).context("Read status")? & 1,
            1
        );

        let reset = self.read_frame().context("Read Reset")?;
        trace!(?reset, "Reset calibration");

        let offset = [
            (set.mag_x.0 + reset.mag_x.0) / 2.0,
            (set.mag_y.0 + reset.mag_y.0) / 2.0,
            (set.mag_z.0 + reset.mag_z.0) / 2.0,
        ];

        self.offset = offset;

        debug!(?offset, "Calibration complete for MCC5982");

        Ok(())
    }

    fn read_reg(&mut self, reg: u8) -> anyhow::Result<u8> {
        let mut output = [0; 2];
        let mut input = [0; 2];

        output[0] = reg | Self::READ;

        self.spi
            .transfer(&mut input, &output)
            .context("Begin read imu frame")?;

        Ok(input[1])
    }

    fn read_raw_frame(&mut self) -> anyhow::Result<[u8; 8]> {
        let mut output = [0; 8];
        let mut input = [0; 8];

        output[0] = Self::REG_XOUT_L | Self::READ;

        self.spi
            .transfer(&mut input, &output)
            .context("Begin read magnetometer frame")?;

        Ok(input)
    }
}

pub struct MagneticCalibration {
    min_x: f32,
    max_x: f32,

    min_y: f32,
    max_y: f32,

    min_z: f32,
    max_z: f32,
}

impl MagneticCalibration {
    pub const HARD_CODED: Self = MagneticCalibration {
        min_x: -0.5869,
        max_x: 0.3240,
        min_y: -0.5970,
        max_y: 0.1972,
        min_z: -0.4848,
        max_z: 0.3264,
    };

    pub fn adjust(&self, measurement: Vec3A) -> Vec3A {
        let x_delta = self.max_x - self.min_x;
        let y_delta = self.max_y - self.min_y;
        let z_delta = self.max_z - self.min_z;

        let x_offset = (self.max_x + self.min_x) / 2.0;
        let y_offset = (self.max_y + self.min_y) / 2.0;
        let z_offset = (self.max_z + self.min_z) / 2.0;

        vec3a(
            (measurement.x - x_offset) * 1.0 / x_delta,
            (measurement.y - y_offset) * 1.0 / y_delta,
            (measurement.z - z_offset) * 1.0 / z_delta,
        )
    }
}
