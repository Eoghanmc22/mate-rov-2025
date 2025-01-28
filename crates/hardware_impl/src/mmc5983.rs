use generic::{
    Hardware,
    device::magnetometer::Magnetometer,
    units::{Gauss, TypedVec3A},
};
use std::{thread, time::Duration};
use tracing::{debug, info, instrument, trace};

use anyhow::{Context, bail};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

pub struct Mcc5983 {
    spi: Spi,
    offset: [f32; 3],
    last_frame: Option<Mcc5983Frame>,
    initialized: bool,
}

#[derive(Debug, Clone)]
pub struct Mcc5983Frame {
    pub mag: TypedVec3A<Gauss>,
}

impl Mcc5983 {
    pub const SPI_BUS: Bus = Bus::Spi1;
    pub const SPI_SELECT: SlaveSelect = SlaveSelect::Ss1;
    pub const SPI_CLOCK: u32 = 10_000_000;

    #[instrument(level = "debug")]
    pub fn new(bus: Bus, slave_select: SlaveSelect, clock_speed: u32) -> anyhow::Result<Self> {
        info!("Setting up MCC5983 (Magnetometer)");

        let spi = Spi::new(bus, slave_select, clock_speed, Mode::Mode0).context("Open spi")?;

        Ok(Self {
            spi,
            offset: [0.0; 3],
            last_frame: None,
            initialized: false,
        })
    }

    // TODO(high): Hard and soft iron calibration?

    #[instrument(level = "trace", skip(self), ret)]
    pub fn read_frame(&mut self) -> anyhow::Result<Mcc5983Frame> {
        let raw = self.read_raw_frame().context("Read raw frame")?;

        // The first byte is junk
        let raw = &raw[1..];

        let raw_mag_native_x =
            ((raw[0] as u32) << 10) | ((raw[1] as u32) << 2) | ((raw[6] as u32 & 0xC0) >> 6);
        let raw_mag_native_y =
            ((raw[2] as u32) << 10) | ((raw[3] as u32) << 2) | ((raw[6] as u32 & 0x30) >> 4);
        let raw_mag_native_z =
            ((raw[4] as u32) << 10) | ((raw[5] as u32) << 2) | ((raw[6] as u32 & 0x0C) >> 2);

        let mag_native_x = (raw_mag_native_x as i32 - 131072) as f32 / 16384.0;
        let mag_native_y = (raw_mag_native_y as i32 - 131072) as f32 / 16384.0;
        let mag_native_z = (raw_mag_native_z as i32 - 131072) as f32 / 16384.0;

        let mag_x = mag_native_y - self.offset[1];
        let mag_y = mag_native_x - self.offset[0];
        let mag_z = mag_native_z - self.offset[2];

        let frame = Mcc5983Frame {
            mag: TypedVec3A::<Gauss>::new_xyz(mag_x, mag_y, mag_z),
        };

        self.last_frame = Some(frame.clone());

        Ok(frame)
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
            (set.mag.0.y + reset.mag.0.y) / 2.0,
            (set.mag.0.x + reset.mag.0.x) / 2.0,
            (set.mag.0.z + reset.mag.0.z) / 2.0,
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
        assert!(self.initialized);

        let mut output = [0; 8];
        let mut input = [0; 8];

        output[0] = Self::REG_XOUT_L | Self::READ;

        self.spi
            .transfer(&mut input, &output)
            .context("Begin read magnetometer frame")?;

        Ok(input)
    }
}

impl Hardware for Mcc5983 {
    fn init(&mut self) -> anyhow::Result<()> {
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

        self.initialized = true;

        Ok(())
    }

    fn poll(&mut self) -> anyhow::Result<()> {
        self.read_frame().context("Read frame")?;

        Ok(())
    }

    fn fastest_polling_interval(&self) -> anyhow::Result<Option<Duration>> {
        Ok(Some(Duration::from_secs_f32(1.0 / 100.0)))
    }

    fn suggested_polling_interval(&self) -> anyhow::Result<Duration> {
        Ok(Duration::from_secs_f32(1.0 / 100.0))
    }
}

impl Magnetometer for Mcc5983 {
    fn read_magnetometer(&self) -> anyhow::Result<TypedVec3A<Gauss>> {
        if let Some(last_frame) = &self.last_frame {
            return Ok(last_frame.mag.clone());
        }

        bail!("Mcc5983 Read before poll")
    }
}
