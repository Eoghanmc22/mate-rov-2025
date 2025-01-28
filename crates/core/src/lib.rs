//! Code shared between both the surface and robot projects
#![feature(try_blocks, hash_extract_if)]

use bevy::{
    app::{PluginGroup, PluginGroupBuilder},
    prelude::*,
};
use ctrlc::CtrlCPlugin;
use ecs_sync::{
    AppReplicateExt, NetId, Replicate, Singleton,
    apply_changes::ChangeApplicationPlugin,
    detect_changes::ChangeDetectionPlugin,
    sync::{Latency, SyncPlugin, SyncRole},
};
use error::ErrorPlugin;
use over_run::OverRunPligin;

pub mod ctrlc;
pub mod ecs_sync;
pub mod error;
pub mod over_run;
pub mod serialization;

pub struct CommonPlugins {
    pub name: String,
    pub role: SyncRole,
}

#[derive(Resource, Debug, Clone)]
pub struct InstanceName(pub String);

impl PluginGroup for CommonPlugins {
    fn build(self) -> PluginGroupBuilder {
        let name = self.name;

        PluginGroupBuilder::start::<Self>()
            .add(move |app: &mut App| {
                app.insert_resource(InstanceName(name.clone()));

                app.register_type::<NetId>()
                    .register_type::<Singleton>()
                    .register_type::<Replicate>()
                    .register_type::<Latency>();
                // .register_type::<Peer>();

                app.replicate::<Transform>().replicate_reflect::<Name>();
            })
            .add(SyncPlugin(self.role))
            // .add(CommunicationTypes)
            .add(ChangeDetectionPlugin)
            .add(ChangeApplicationPlugin)
            .add(CtrlCPlugin)
            .add(ErrorPlugin)
            .add(OverRunPligin)
    }
}
