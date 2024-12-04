// TODO: Register
#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct DepthFrame {
    pub depth: Meters,
    pub altitude: Meters,
    pub pressure: Mbar,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct DepthSettings {
    pub sea_level: Mbar,
    pub fluid_density: f32,
}

#[derive(Event, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq)]
pub struct CalibrateSeaLevel;

pub fn pressure_to_depth(pressure: Mbar, density: f32, sea_level: f32) -> Meters {
    Meters(((pressure.0 - sea_level) * 100.0) / (density * 9.80665))
}

pub fn pressure_to_altitude(pressure: Mbar, sea_level: f32) -> Meters {
    Meters((1.0 - f32::powf(pressure.0 / sea_level, 0.190284)) * 145366.45 * 0.3048)
}
