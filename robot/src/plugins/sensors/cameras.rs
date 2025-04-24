pub mod camera_handler;

use std::{net::SocketAddr, thread};

use anyhow::Context;
use bevy::{app::AppExit, prelude::*};
use camera_handler::{CameraHandler, FatalError};
use common::{
    bundles::CameraBundle,
    components::{CameraDefinition, RobotId},
    ecs_sync::{NetId, Replicate},
    error::{self, Errors},
    events::ResyncCameras,
    sync::Peer,
};
use crossbeam::channel::{self, Receiver, Sender};
use tracing::{span, Level};

use crate::{
    config::RobotConfig,
    plugins::core::robot::{LocalRobot, LocalRobotMarker},
};

// TODO(low): Use multicast udp
pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, start_camera_thread.pipe(error::handle_errors));
        app.add_systems(PreUpdate, read_new_data);
        app.add_systems(Update, handle_peers);
        app.add_systems(Last, shutdown);
    }
}

#[derive(Resource)]
struct CameraChannels(Sender<CameraEvent>, Receiver<Vec<CameraBundle>>);

enum CameraEvent {
    NewPeer(SocketAddr),
    LostPeer,
    // TODO(low): Should we resync cameras on an interval?
    Resync,
    Shutdown,
}

fn start_camera_thread(
    mut cmds: Commands,
    errors: Res<Errors>,
    robot: Res<LocalRobot>,
    config: Res<RobotConfig>,
) -> anyhow::Result<()> {
    let (tx_events, rx_events) = channel::bounded(10);
    let (tx_camreas, rx_cameras) = channel::bounded(10);

    info!("Setting up cameras");

    let _ = tx_events.send(CameraEvent::Resync);

    cmds.insert_resource(CameraChannels(tx_events, rx_cameras));

    let errors = errors.0.clone();
    let robot = RobotId(robot.net_id);
    let config = config.camera_config.clone();

    thread::Builder::new()
        .name("Camera Thread".to_owned())
        .spawn(move || -> Result<(), FatalError> {
            let _span = span!(Level::INFO, "Camera manager").entered();

            let error_handler = |error| {
                let _ = errors.send(error);
            };
            let mut handler = CameraHandler::new(tx_camreas, robot, config, &error_handler);

            for event in rx_events {
                match event {
                    CameraEvent::NewPeer(socket_addr) => {
                        handler.update_peer(Some(socket_addr))?;
                    }
                    CameraEvent::LostPeer => {
                        handler.update_peer(None)?;
                    }
                    CameraEvent::Resync => {
                        handler.restart_streams()?;
                    }
                    CameraEvent::Shutdown => {
                        handler.kill_streams()?;
                        return Ok(());
                    }
                }
            }

            Err(FatalError::ChannelDisconnected)
        })
        .context("Spawn thread")?;

    Ok(())
}

fn handle_peers(
    channels: Res<CameraChannels>,
    mut disconnected: RemovedComponents<Peer>,
    connected: Query<&Peer, Changed<Peer>>,
    mut resync_events: EventReader<ResyncCameras>,
) {
    let res: Result<(), crossbeam::channel::SendError<_>> = try {
        for _resync in resync_events.read() {
            channels.0.send(CameraEvent::Resync)?;
        }

        for _disconnection in disconnected.read() {
            channels.0.send(CameraEvent::LostPeer)?;
        }

        for peer in connected.iter() {
            channels.0.send(CameraEvent::NewPeer(peer.addrs))?;
        }
    };

    if let Err(_) = res {
        error!("Camera thread dead");
    }
}

// TODO(low): Only update the cameras that changed
fn read_new_data(
    mut cmds: Commands,
    channels: Res<CameraChannels>,
    robot: Query<(Entity, &NetId), With<LocalRobotMarker>>,
    cameras: Query<(Entity, &RobotId), With<CameraDefinition>>,
) {
    let mut new_cameras = None;
    for camera_update in channels.1.try_iter() {
        new_cameras = Some(camera_update);
    }

    if let Some(new_cameras) = new_cameras {
        let (_robot, id) = robot.single();

        for (entity, camera_robot) in &cameras {
            if camera_robot.0 == *id {
                cmds.entity(entity).despawn();
            }
        }

        for camera in new_cameras {
            cmds.spawn((camera, Replicate));
        }
    }
}

fn shutdown(channels: Res<CameraChannels>, mut exit: EventReader<AppExit>) {
    for _event in exit.read() {
        let _ = channels.0.send(CameraEvent::Shutdown);
    }
}
