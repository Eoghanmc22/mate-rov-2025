use bevy::{app::PluginGroupBuilder, prelude::PluginGroup};

pub mod hw_stat;
pub mod voltage;

pub struct MonitorPlugins;

impl PluginGroup for MonitorPlugins {
    fn build(self) -> PluginGroupBuilder {
        let builder = PluginGroupBuilder::start::<Self>().add(hw_stat::HwStatPlugin);

        #[cfg(rpi)]
        let builder = builder.add(voltage::VoltagePlugin);

        builder
    }
}
