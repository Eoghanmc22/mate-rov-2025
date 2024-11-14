pub mod trajectory;
pub mod ui;
pub mod waterlinked;
pub mod waterlinked_api;

use bevy::diagnostic::EntityCountDiagnosticsPlugin;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::diagnostic::LogDiagnosticsPlugin;
use bevy::prelude::PluginGroup;
use bevy_tokio_tasks::TokioTasksPlugin;
use common::sync::SyncRole;
use common::CommonPlugins;
use std::time::Duration;
use trajectory::TrajectoryPlugin;
use ui::EguiUiPlugin;
use waterlinked::WaterlinkedPlugin;

use bevy::{app::App, color::Color, prelude::ClearColor, DefaultPlugins};
use common::over_run::OverRunSettings;
use tracing::info;

pub const DARK_MODE: bool = false;

// TODO: - Compass impl in robot
//       - Go to relative coordinate UI and controller impl
//       - Figure out how to map waterlinked position into robot space
//       - eventually a kalman filter?
fn main() {
    info!("---------- Starting Autonomous Controller ----------");

    // FIXME(high): Times out when focus is lost
    App::new()
        .insert_resource(OverRunSettings {
            max_time: Duration::from_secs_f32(1.0 / 60.0),
            tracy_frame_mark: false,
        })
        .insert_resource(if DARK_MODE {
            ClearColor(Color::srgb_u8(33, 34, 37))
        } else {
            ClearColor(Color::srgb_u8(240, 238, 233))
        })
        .add_plugins((
            // Bevy Core
            DefaultPlugins.build().disable::<bevy::audio::AudioPlugin>(),
            // Diagnostics
            (
                LogDiagnosticsPlugin::default(),
                EntityCountDiagnosticsPlugin,
                FrameTimeDiagnosticsPlugin,
            ),
            // MATE
            (
                CommonPlugins {
                    name: "Autonomous Controller".to_owned(),
                    role: SyncRole::Client,
                },
                EguiUiPlugin,
                WaterlinkedPlugin,
                TrajectoryPlugin,
            ),
            // 3rd Party
            (TokioTasksPlugin::default()),
        ))
        // .add_systems(Startup, (spawn_camera, request_slippy_tiles))
        // .add_systems(Update, display_slippy_tiles)
        .run();

    info!("---------- Autonomous Controller Exited Cleanly ----------");
}
