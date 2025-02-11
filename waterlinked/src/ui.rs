use bevy::{
    app::{App, Plugin, Startup, Update},
    core::Name,
    math::{vec3a, Quat},
    prelude::{Commands, Entity, EventWriter, Local, Query, Res, ResMut, With, World},
    reflect::List,
};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::{
    components::{Robot, RobotId},
    sync::{ConnectToPeer, DisconnectPeer, MdnsPeers, Peer},
};
use egui::{CentralPanel, Color32, PointerButton, Visuals};
use egui_plot::{Line, MarkerShape, Plot, PlotItem, PlotPoint, PlotPoints, Points};
use tracing::{error, info, warn};

use crate::{
    trajectory::{CurrentPose, Pose, TargetPose},
    DARK_MODE,
};

pub struct EguiUiPlugin;

impl Plugin for EguiUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, set_style);
        app.add_plugins(EguiPlugin).add_systems(Update, main_pane);
    }
}

fn set_style(mut contexts: EguiContexts) {
    contexts.ctx_mut().set_visuals(if DARK_MODE {
        Visuals::dark()
    } else {
        Visuals::light()
    });
}

fn main_pane(
    mut host: Local<String>,
    mut position_history: Local<Vec<[f64; 2]>>,

    mut cmds: Commands,
    mut contexts: EguiContexts,
    runtime: ResMut<TokioTasksRuntime>,

    robots: Query<
        (
            Entity,
            &Name,
            Option<&CurrentPose>,
            Option<&TargetPose>,
            &RobotId,
        ),
        With<Robot>,
    >,
    mdns_peers: Option<Res<MdnsPeers>>,
    peers: Query<&Peer>,

    mut disconnect: EventWriter<DisconnectPeer>,
) {
    CentralPanel::default().show(contexts.ctx_mut(), |ui| {
        if let Ok((robot, name, current_pose, target_pose, robot_id)) = robots.get_single() {
            ui.horizontal(|ui| {
                ui.label(format!("Connected to {}", name.as_str()));
                if ui.button("Disconnect").clicked() {
                    for peer in &peers {
                        disconnect.send(DisconnectPeer(peer.token));
                    }
                }
            });
            if let Some(current_pose) = current_pose {
                let pos = current_pose.0.position;
                ui.label(format!(
                    "Current Location: x: {:.02}, y: {:.02}, z: {:.02}",
                    pos.x, pos.y, pos.z,
                ));
            } else {
                ui.label("Current Location: None");
            }
            if let Some(target_pose) = target_pose {
                let pos = target_pose.0.position;
                ui.horizontal(|ui| {
                    ui.label(format!(
                        "Target Location: x: {:.02}, y: {:.02}, z: {:.02}",
                        pos.x, pos.y, pos.z,
                    ));
                    if ui.button("Clear").clicked() {
                        cmds.entity(robot).remove::<TargetPose>();
                    }
                });
            } else {
                ui.label("Target Location: None");
            }
            if let (Some(current_pose), Some(target_pose)) = (current_pose, target_pose) {
                let current_pos = current_pose.0.position;
                let target_pos = target_pose.0.position;
                let delta = target_pos - current_pos;

                ui.label(format!(
                    "{:.02} from target ({:.02}, {:.02}, {:.02})",
                    delta.length(),
                    delta.x,
                    delta.y,
                    delta.z
                ));
            }

            // Position plot
            if let Some(current_pose) = current_pose {
                let current_pos = current_pose.0.position;
                position_history.push([current_pos.x as f64, current_pos.y as f64]);

                let response = Plot::new("Position Track")
                    .data_aspect(1.0)
                    .include_x(0.0)
                    .include_y(0.0)
                    .width(500.0)
                    .height(500.0)
                    .show(ui, |ui| {
                        ui.line(Line::new((*position_history).clone()).name("Track"));
                        ui.points(
                            Points::new([current_pos.x as f64, current_pos.y as f64])
                                .name("Current Position")
                                .shape(MarkerShape::Circle)
                                .color(Color32::BLUE)
                                .radius(3.0),
                        );

                        if let Some(target_pose) = target_pose {
                            let target_pos = target_pose.0.position;
                            ui.points(
                                Points::new([target_pos.x as f64, target_pos.y as f64])
                                    .name("Target Position")
                                    .shape(MarkerShape::Up)
                                    .color(Color32::DARK_GREEN)
                                    .radius(5.0),
                            );
                        }
                    });

                if response
                    .response
                    .double_clicked_by(PointerButton::Secondary)
                {
                    let mouse = response.response.hover_pos();
                    if let Some(mouse) = mouse {
                        let position = response.transform.value_from_position(mouse);

                        cmds.entity(robot).insert(TargetPose(Pose {
                            position: vec3a(position.x as f32, position.y as f32, 0.0),
                            rotation: Quat::IDENTITY,
                        }));
                    }
                }
            } else {
                position_history.clear();
            }
        } else {
            ui.horizontal(|ui| {
                ui.label("Connect To:");
                let line_response = ui.text_edit_singleline(&mut *host);
                let button_response = ui.button("Connect");

                if line_response.lost_focus() || button_response.clicked() {
                    let host = host.clone();
                    runtime.spawn_background_task(|mut ctx| async move {
                        let resolve = tokio::net::lookup_host(host).await;
                        let addrs = resolve.ok().and_then(|mut it| it.next());

                        if let Some(addrs) = addrs {
                            ctx.run_on_main_thread(move |ctx| {
                                let world = ctx.world;
                                let count = world.query::<&Robot>().iter(world).count();

                                if count == 0 {
                                    info!("Peer ip resolved to {:?}", addrs);
                                    world.send_event(ConnectToPeer(addrs));
                                } else {
                                    warn!("Already connected to peer");
                                }
                            })
                            .await;
                        } else {
                            error!("Could not resolve host");
                        }
                    });
                }
            });

            if let Some(peers) = mdns_peers {
                let peers = &peers.0;

                if !peers.is_empty() {
                    ui.add_space(15.0);

                    ui.heading("Peers:");

                    for peer in peers.values() {
                        let name = peer
                            .info
                            .get_fullname()
                            .split('.')
                            .next()
                            .unwrap_or("Unknown");
                        let host = peer.info.get_hostname();

                        ui.label(format!("{}@{}local", name, host));

                        ui.indent(peer.info.get_fullname(), |ui| {
                            for addrs in &peer.addresses {
                                let addrs = *addrs;

                                if ui.button(format!("{}", addrs.ip())).clicked() {
                                    cmds.queue(move |world: &mut World| {
                                        world.send_event(ConnectToPeer(addrs));
                                    });
                                }
                            }
                        });
                    }
                }
            }
        }
    });
}
