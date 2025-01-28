use crate::units::{Gauss, TypedVec3A};

pub trait Magnetometer {
    fn read_magnetometer(&self) -> anyhow::Result<TypedVec3A<Gauss>>;
}
