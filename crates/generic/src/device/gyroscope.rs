use crate::units::{Dps, TypedVec3A};

pub trait Gyroscope {
    fn read_gyroscope(&self) -> anyhow::Result<TypedVec3A<Dps>>;
}
