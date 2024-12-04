// TODO: Register
#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct GyroFrame {
    pub gyro_x: Dps,
    pub gyro_y: Dps,
    pub gyro_z: Dps,

    pub accel_x: GForce,
    pub accel_y: GForce,
    pub accel_z: GForce,

    pub tempature: Celsius,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct AccelerometerFrame {
    pub accel_x: GForce,
    pub accel_y: GForce,
    pub accel_z: GForce,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct MagneticFrame {
    pub mag_x: Gauss,
    pub mag_y: Gauss,
    pub mag_z: Gauss,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct DepthFrame {
    pub depth: Meters,
    pub altitude: Meters,
    pub pressure: Mbar,
}

#[derive(Component, Serialize, Deserialize, Reflect, Debug, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct TemperatureFrame {
    pub tempature: Celsius,
}

pub fn register_types(app: &mut App) {
    app.register_type::<GyroFrame>()
        .register_type::<AccelerometerFrame>()
        .register_type::<MagneticFrame>()
        .register_type::<DepthFrame>()
        .register_type::<TemperatureFrame>();
}
