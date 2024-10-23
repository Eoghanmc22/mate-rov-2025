use std::{collections::HashMap as StdHashMap, hash::BuildHasher};

pub type StableHashMap<K, V> = StdHashMap<K, V, StableState>;

#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "reflect", derive(bevy_reflect::Reflect))]
pub struct StableState;

impl BuildHasher for StableState {
    type Hasher = ahash::AHasher;

    fn build_hasher(&self) -> Self::Hasher {
        ahash::RandomState::with_seeds(0, 0, 0, 0).build_hasher()
    }
}
