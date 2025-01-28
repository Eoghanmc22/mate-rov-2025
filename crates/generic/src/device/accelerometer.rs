use crate::units::{GForce, TypedVec3A};

pub trait Accelerometer {
    fn read_acceleration(&self) -> anyhow::Result<TypedVec3A<GForce>>;
}
