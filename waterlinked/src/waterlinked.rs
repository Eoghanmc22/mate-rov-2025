use std::time::Duration;

use anyhow::Context;
use bevy::{
    app::{Plugin, PreUpdate, Startup, Update},
    math::vec3a,
    prelude::{App, Commands, Entity, Event, EventReader, Query, ResMut, With},
};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::components::{Orientation, Robot};
use tracing::{error, warn};

use crate::{
    trajectory::{CurrentPose, Pose},
    waterlinked_api::{wl_to_mate_coords, Location, WaterLinked},
};

pub struct WaterlinkedPlugin;

impl Plugin for WaterlinkedPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<WaterlinkedLocationEvent>();

        app.add_systems(Startup, start_task);
        app.add_systems(PreUpdate, pose_updater);
    }
}

#[derive(Event, Debug)]
pub struct WaterlinkedLocationEvent(pub Location);

fn start_task(runtime: ResMut<TokioTasksRuntime>) {
    runtime.spawn_background_task(|mut ctx| async move {
        let mut interval = tokio::time::interval(Duration::from_secs_f64(1.0 / 4.0));
        let api = WaterLinked::new("https://demo.waterlinked.com/".try_into().unwrap());

        loop {
            interval.tick().await;

            let location = match api.get_location().await.context("Get Location") {
                Ok(location) => location,
                Err(err) => {
                    error!("Waterlinked error: {err:?}");
                    continue;
                }
            };

            ctx.run_on_main_thread(|ctx| ctx.world.send_event(WaterlinkedLocationEvent(location)))
                .await;
        }
    });
}

fn pose_updater(
    mut cmds: Commands,
    robot: Query<(Entity, Option<&Orientation>), With<Robot>>,
    mut reader: EventReader<WaterlinkedLocationEvent>,
) {
    let Ok((robot, orientation)) = robot.get_single() else {
        return;
    };

    for event in reader.read() {
        let Location {
            position_valid,
            x,
            y,
            z,
            ..
        } = event.0.clone();

        let (x, y, z) = wl_to_mate_coords(x, y, z);

        if position_valid {
            cmds.entity(robot).insert(CurrentPose(Pose {
                position: vec3a(x, y, z),
                rotation: orientation.map(|it| it.0).unwrap_or_default(),
            }));
        } else {
            warn!("Recieved bad UGPS update");
        }
    }
}
