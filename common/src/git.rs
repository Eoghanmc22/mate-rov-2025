use std::borrow::Cow;

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

// Currently the syncronization of this component is special in that it gets its own packet
#[derive(Component, Reflect, Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct GitMetadata {
    pub branch: Cow<'static, str>,
    pub commit_message: Cow<'static, str>,
    pub commit_hash: Cow<'static, str>,
    pub commit_date: Cow<'static, str>,
    pub commit_timestamp: Cow<'static, str>,
    pub dirty: Cow<'static, str>,
}

impl GitMetadata {
    pub fn new() -> Option<Self> {
        let branch = option_env!("VERGEN_GIT_BRANCH")?.into();
        let commit_message = option_env!("VERGEN_GIT_COMMIT_MESSAGE")?.into();
        let commit_hash = option_env!("VERGEN_GIT_SHA")?.into();
        let commit_date = option_env!("VERGEN_GIT_COMMIT_DATE")?.into();
        let commit_timestamp = option_env!("VERGEN_GIT_COMMIT_TIMESTAMP")?.into();
        let dirty = option_env!("VERGEN_GIT_DIRTY")?.into();

        Some(Self {
            branch,
            commit_message,
            commit_hash,
            commit_date,
            commit_timestamp,
            dirty,
        })
    }
}
