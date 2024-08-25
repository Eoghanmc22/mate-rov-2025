use std::{collections::HashMap as StdHashMap, hash::BuildHasher};

pub type StableHashMap<K, V> = StdHashMap<K, V, StableState>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StableState;

impl BuildHasher for StableState {
    type Hasher = ahash::AHasher;

    fn build_hasher(&self) -> Self::Hasher {
        ahash::RandomState::with_seeds(1, 2, 3, 4).build_hasher()
    }
}

impl Default for StableState {
    fn default() -> StableState {
        Self
    }
}
