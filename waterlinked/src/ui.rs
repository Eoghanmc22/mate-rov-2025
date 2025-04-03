use core::f32;

use bevy::{
    app::{App, Plugin, Startup, Update},
    core::Name,
    ecs::{
        entity,
        world::{EntityWorldMut, FromWorld},
    },
    math::{vec3a, EulerRot, Quat, Vec3},
    prelude::{Commands, Entity, EventWriter, Local, Query, Res, ResMut, With, World},
    time::{Real, Time},
};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::{
    components::{
        FilteredPose, Orientation, OrientationTarget, Pose, RawPose, Robot, RobotId, TargetPose,
    },
    sync::{ConnectToPeer, DisconnectPeer, MdnsPeers, Peer},
};
use egui::{CentralPanel, Color32, PointerButton, Slider, Visuals};
use egui_plot::{Line, MarkerShape, Plot, PlotItem, PlotPoint, PlotPoints, Points};
use tokio::sync::oneshot::error;
use tracing::{error, info, warn};

use crate::{
    learn_compass::{MagneticData, ResetMagneticLog},
    orientation::OrientationState,
    waterlinked::WaterlinkedAngleOffset,
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
    mut position_history: Local<(Vec<[f64; 2]>, Vec<[f64; 2]>)>,
    mut orientation_norm_history: Local<Vec<(f32, Vec3)>>,

    mut cmds: Commands,
    mut contexts: EguiContexts,
    runtime: ResMut<TokioTasksRuntime>,

    mag_data: Res<MagneticData>,
    mut angle_offset: ResMut<WaterlinkedAngleOffset>,

    robots: Query<
        (
            Entity,
            &Name,
            Option<&RawPose>,
            Option<&FilteredPose>,
            Option<&TargetPose>,
            &Orientation,
            Option<&OrientationTarget>,
            Option<&OrientationState>,
            &RobotId,
        ),
        With<Robot>,
    >,
    mdns_peers: Option<Res<MdnsPeers>>,
    peers: Query<&Peer>,

    time: Res<Time<Real>>,

    mut disconnect: EventWriter<DisconnectPeer>,
) {
    CentralPanel::default().show(contexts.ctx_mut(), |ui| {
        if let Ok((
            robot,
            name,
            current_pose,
            filtered_pose,
            target_pose,
            orientation,
            orientation_target,
            orientation_state,
            robot_id,
        )) = robots.get_single()
        {
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

            let mut angle_offset_deg = angle_offset.0.to_euler(EulerRot::ZXY).0.to_degrees();
            let angle_offset_deg_copy = angle_offset_deg;
            ui.add(Slider::new(&mut angle_offset_deg, -180.0..=180.0).text("Angle Offset"));
            if angle_offset_deg != angle_offset_deg_copy {
                angle_offset.0 =
                    Quat::from_euler(EulerRot::ZXY, angle_offset_deg.to_radians(), 0.0, 0.0);
            }

            // Position plot
            if let Some(current_pose) = current_pose {
                let current_pos = current_pose.0.position;
                position_history
                    .0
                    .push([current_pos.x as f64, current_pos.y as f64]);
                if let Some(filtered_pose) = filtered_pose {
                    let filtered_pose = filtered_pose.pose.position;

                    position_history
                        .1
                        .push([filtered_pose.x as f64, filtered_pose.y as f64]);
                }

                let response = Plot::new("Position Track")
                    .data_aspect(1.0)
                    .include_x(0.0)
                    .include_y(0.0)
                    .width(500.0)
                    .height(500.0)
                    .show(ui, |ui| {
                        ui.line(Line::new((position_history.0).clone()).name("Raw Track"));
                        ui.line(Line::new((position_history.1).clone()).name("Filtered Track"));
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
                position_history.0.clear();
                position_history.1.clear();
            }

            if ui.button("Clear Map").clicked() {
                position_history.0.clear();
                position_history.1.clear();
            }

            ui.label(format!("{mag_data:.04?}"));
            if ui.button("Reset Mag Log").clicked() {
                cmds.queue(|world: &mut World| {
                    world.send_event(ResetMagneticLog);
                });
            }

            if orientation_state.is_some() {
                if let Some(orientation_target) = orientation_target {
                    let error = orientation_target.0 * orientation.0.inverse();
                    let error = error.to_scaled_axis();

                    fn normalize_angle(angle: f32) -> f32 {
                        let wrapped_angle = modf(angle, f32::consts::TAU);
                        if wrapped_angle > f32::consts::PI {
                            wrapped_angle - f32::consts::TAU
                        } else {
                            wrapped_angle
                        }
                    }

                    fn modf(a: f32, b: f32) -> f32 {
                        (a % b + b) % b
                    }

                    let error = error.normalize_or_zero() * normalize_angle(error.length());

                    // let error = error.to_euler(EulerRot::ZYX);

                    // orientation_norm_history
                    //     .push((time.elapsed_secs(), Vec3::new(error.2, error.1, error.0)));
                    orientation_norm_history.push((time.elapsed_secs(), error));
                }

                Plot::new("Orientation error norm")
                    .width(ui.available_width())
                    .height(500.0)
                    .show(ui, |ui| {
                        ui.line(
                            Line::new(
                                orientation_norm_history
                                    .iter()
                                    .map(|(time, pos)| [*time as _, pos.x as _])
                                    .collect::<Vec<[f64; 2]>>(),
                            )
                            .name("X rot norm"),
                        );
                        ui.line(
                            Line::new(
                                orientation_norm_history
                                    .iter()
                                    .map(|(time, pos)| [*time as _, pos.y as _])
                                    .collect::<Vec<[f64; 2]>>(),
                            )
                            .name("Y rot norm"),
                        );
                        ui.line(
                            Line::new(
                                orientation_norm_history
                                    .iter()
                                    .map(|(time, pos)| [*time as _, pos.z as _])
                                    .collect::<Vec<[f64; 2]>>(),
                            )
                            .name("Z rot norm"),
                        );
                    });

                if ui.button("Save").clicked() {
                    let mut writer = csv::Writer::from_path("quat_norm.csv").unwrap();

                    #[derive(serde::Serialize)]
                    struct Data {
                        time: f32,
                        x: f32,
                        y: f32,
                        z: f32,
                    }

                    for entry in &orientation_norm_history {
                        let data = Data {
                            time: entry.0,
                            x: entry.1.x,
                            y: entry.1.y,
                            z: entry.1.z,
                        };
                        writer.serialize(&data).unwrap();
                    }

                    writer.flush().unwrap();
                }

                if ui.button("Stop").clicked() {
                    cmds.entity(robot).remove::<OrientationState>();
                    cmds.entity(robot).remove::<OrientationTarget>();
                }
            } else if ui.button("Spin!").clicked() {
                orientation_norm_history.clear();

                cmds.entity(robot)
                    .queue(|mut entity_world_mut: EntityWorldMut| {
                        let orientation_state =
                            entity_world_mut.world_scope(OrientationState::from_world);
                        entity_world_mut.insert(orientation_state);
                    });
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
