use bevy::{app::PluginGroupBuilder, prelude::PluginGroup};

pub mod robot;
pub mod state;
pub mod stats;

pub struct CorePlugins;

impl PluginGroup for CorePlugins {
    fn build(self) -> PluginGroupBuilder {
        PluginGroupBuilder::start::<Self>()
            .add(robot::RobotPlugin)
            .add(state::StatePlugin)
            .add(stats::StatisticsPlugin)
    }
}
