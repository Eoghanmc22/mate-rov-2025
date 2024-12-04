// TODO: Register
#[derive(Component, Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
#[reflect(SerdeAdapter, Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct Leak(pub bool);
