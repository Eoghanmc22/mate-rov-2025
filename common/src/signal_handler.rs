use std::{
    io,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};

use bevy::{app::AppExit, prelude::*};
use signal_hook::consts::*;

use crate::error::ErrorEvent;

use super::error;

pub struct SignalPlugin;

impl Plugin for SignalPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_handler.pipe(error::handle_errors));
        app.add_systems(PreUpdate, check_handler);
    }
}

#[derive(Resource)]
struct SignalState(Arc<AtomicBool>);

fn setup_handler(mut cmds: Commands, mut errors: EventWriter<ErrorEvent>) -> anyhow::Result<()> {
    let do_shutdown = Arc::new(AtomicBool::new(false));

    let res: Result<(), io::Error> = try {
        signal_hook::flag::register_conditional_shutdown(SIGTERM, 1, do_shutdown.clone())?;
        signal_hook::flag::register_conditional_shutdown(SIGINT, 1, do_shutdown.clone())?;
        signal_hook::flag::register(SIGTERM, do_shutdown.clone())?;
        signal_hook::flag::register(SIGINT, do_shutdown.clone())?;
    };

    if let Err(err) = res {
        errors.send(
            anyhow::anyhow!(err)
                .context("Could not register signal handlers")
                .into(),
        );
    }

    cmds.insert_resource(SignalState(do_shutdown));

    Ok(())
}

fn check_handler(state: Res<SignalState>, mut exit: EventWriter<AppExit>) {
    if state.0.load(Ordering::Relaxed) {
        exit.send(AppExit::Success);
    }
}
