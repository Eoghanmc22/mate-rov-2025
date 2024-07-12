use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfaceDefinition {
    pub name: String,
    #[serde(flatten)]
    pub hardware: HardwareDefinition,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "hardware")]
pub enum HardwareDefinition {
    #[serde(rename = "pca9685_i2c")]
    Pca9685(Pca9685Definition),
    #[serde(rename = "ads1115_i2c")]
    Ads1115(Ads1115Definition),
    #[serde(rename = "bluerov_powersense_adc")]
    PowerSense(BlueRovPowerSenseDefinition),
    #[serde(rename = "icm20602_spi")]
    Icm20602(Icm20602Definition),
    #[serde(rename = "mmc5983_spi")]
    Mmc5983(Mmc5983Definition),
    #[serde(rename = "ms5937_i2c")]
    Ms5937(Ms5937Definition),
    #[serde(rename = "neopixel_spi")]
    Neopixel(NeopixelDefinition),
    #[serde(rename = "leak_gpio")]
    Leak(LeakDefinition),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct I2cDefinition {
    pub i2c_bus: u32,
    pub i2c_address: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpiDefinition {
    pub spi_bus: u32,
    pub spi_cs: u32,
}

// TODO: Move to Pca9685 Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pca9685Definition {
    #[serde(flatten)]
    pub i2c: I2cDefinition,
    pub enable_gpio: u32,
}

// TODO: Move to Ads1115 Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ads1115Definition {
    #[serde(flatten)]
    pub i2c: I2cDefinition,
}

// TODO: Move to bluerov_powersense Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlueRovPowerSenseDefinition {
    pub adc_name: String,
}

// TODO: Move to icm20602 Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Icm20602Definition {
    #[serde(flatten)]
    pub spi: SpiDefinition,
}

// TODO: Move to mmc5937 Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mmc5983Definition {
    #[serde(flatten)]
    pub spi: SpiDefinition,
}

// TODO: Move to ms5937 Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ms5937Definition {
    #[serde(flatten)]
    pub i2c: I2cDefinition,
    pub fluid_density: f32,
    pub sea_level_pressure: f32,
}

// TODO: Move to neopixel Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NeopixelDefinition {
    #[serde(flatten)]
    pub spi: SpiDefinition,
}

// TODO: Move to leak Module
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LeakDefinition {
    pub gpio: u32,
    pub active_high: bool,
}
