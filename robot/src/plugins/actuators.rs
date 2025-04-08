pub mod depth_hold;
pub mod hardware;
pub mod leds;
pub mod servo;
pub mod stabilize;
pub mod thruster;

use bevy::{app::PluginGroupBuilder, prelude::PluginGroup};

pub struct MovementPlugins;

impl PluginGroup for MovementPlugins {
    fn build(self) -> PluginGroupBuilder {
        let plugins = PluginGroupBuilder::start::<Self>()
            .add(servo::ServoPlugin)
            .add(thruster::ThrusterPlugin)
            .add(stabilize::StabilizePlugin)
            .add(depth_hold::DepthHoldPlugin);

        #[cfg(rpi)]
        let plugins = plugins
            // Plugins depending on robot hardware
            .add(hardware::pwm::PwmOutputPlugin)
            .add(hardware::dc_motor::DcMotorPlugin)
            .add(leds::LedPlugin);

        plugins
    }
}
