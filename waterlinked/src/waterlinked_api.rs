// https://demo.waterlinked.com/swagger/

use anyhow::Context;
use reqwest::Url;
use serde::Deserialize;

pub struct WaterLinked {
    api_endpoint: Url,
    client: reqwest::Client,
}

impl WaterLinked {
    pub fn new(api_endpoint: Url) -> Self {
        let client = reqwest::Client::new();

        Self {
            api_endpoint,
            client,
        }
    }

    pub async fn get_location(&self) -> anyhow::Result<Location> {
        let response: Location = self
            .client
            .get(
                self.api_endpoint
                    .join("/api/v1/position/acoustic/filtered")
                    .context("Build url")?,
            )
            .send()
            .await
            .context("Send request")?
            .json()
            .await
            .context("Await Json")?;

        Ok(response)
    }

    pub async fn get_locator_gps(&self) -> anyhow::Result<GpsFix> {
        let response: GpsFix = self
            .client
            .get(
                self.api_endpoint
                    .join("/api/v1/position/global")
                    .context("Build url")?,
            )
            .send()
            .await
            .context("Send request")?
            .json()
            .await
            .context("Await Json")?;

        Ok(response)
    }

    pub async fn get_surface_gps(&self) -> anyhow::Result<GpsFix> {
        let response: GpsFix = self
            .client
            .get(
                self.api_endpoint
                    .join("/api/v1/position/master")
                    .context("Build url")?,
            )
            .send()
            .await
            .context("Send request")?
            .json()
            .await
            .context("Await Json")?;

        Ok(response)
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct Location {
    pub position_valid: bool,

    pub receiver_distance: Vec<f32>,
    pub receiver_nsd: Vec<f32>,
    pub receiver_rssi: Vec<f32>,
    pub receiver_valid: Vec<f32>,

    pub std: f32,

    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Deserialize, Clone)]
pub struct GpsFix {
    /// Course over ground, degrees (-1 for no data)
    pub cog: f32,
    /// Not sure what type this should be (0 for no data)
    pub fix_quality: f32,
    /// Horizontal dilution of precision. -1 means no data.
    pub hdop: f32,
    /// Current Latitude
    pub lat: f32,
    /// Current Longitude
    pub lon: f32,
    /// Number of satellites. -1 means no data.
    pub numsats: u32,
    /// Current orientation/compass heading (degrees). -1 means no data.
    pub orientation: f32,
    /// Speed over ground (km/h). -1 means no data
    pub sog: f32,
}

pub fn wl_to_mate_coords(x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    // WL: +X: Forward, +Y: Right, +Z: Down
    // MATE: +X: Right, +Y: Forwards, +Z: Up

    (y, x, -z)
}
