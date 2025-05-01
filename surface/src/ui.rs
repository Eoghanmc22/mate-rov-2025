use std::{
    collections::{hash_map::Entry, BTreeMap, VecDeque},
    time::Duration,
};

use ahash::HashMap;
use bevy::{app::AppExit, math::vec3a, prelude::*};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::{
    bundles::MovementContributionBundle,
    components::{
        ActualMovement, Armed, CameraDefinition, CurrentDraw, DepthMeasurement, DepthTarget,
        DisableMovementApi, GenericMotorId, MeasuredVoltage, MotorRawSignalRange, MotorSignal,
        MovementAxisMaximums, MovementContribution, OrientationTarget, PidController, PidResult,
        Robot, RobotId, SystemCpuTotal, SystemLoadAverage, SystemMemory, SystemTemperatures,
        TargetMovement, TempertureMeasurement, ThrusterDefinition,
    },
    ecs_sync::{NetId, Replicate},
    events::{CalibrateSeaLevel, ResetServos, ResetYaw, ResyncCameras},
    sync::{ConnectToPeer, DisconnectPeer, Latency, MdnsPeers, Peer},
    types::units::Amperes,
};
use egui::{
    load::SizedTexture, text::LayoutJob, widgets, Align, Color32, Id, Label, Layout, RichText,
    Sense, TextBuffer, TextFormat, Visuals,
};
use egui_plot::{Line, Plot, PlotPoint};
use leafwing_input_manager::input_map::InputMap;
use motor_math::{glam::MovementGlam, solve::reverse::Axis};
use tokio::net::lookup_host;

use crate::{
    attitude::OrientationDisplay,
    input::{Action, InputInterpolation, InputMarker, SelectedServo},
    photosphere::{PhotoSphere, RotatePhotoSphere, SpawnPhotoSphere},
    video_display_2d_master::VideoMasterMarker,
    video_pipelines::VideoPipelines,
    video_stream::{VideoProcessorFactory, VideoThread},
    DARK_MODE,
};

pub struct EguiUiPlugin;

impl Plugin for EguiUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, set_style);
        app.add_plugins(EguiPlugin).add_systems(
            Update,
            // TODO: create a system set for `.after(topbar)` and move each
            // ui component to a seperate module
            (
                topbar,
                hud.after(topbar),
                // TODO: Move to photosphere.rs
                photosphere.after(topbar),
                movement_control.after(topbar),
                pid_helper.after(topbar),
                movement_debug.after(topbar),
                current_draw_debug.after(topbar),
                pwm_control
                    .after(topbar)
                    .run_if(resource_exists::<PwmControl>),
                cleanup_pwm_control
                    .after(topbar)
                    .run_if(resource_removed::<PwmControl>),
                timer.after(topbar).run_if(resource_exists::<TimerUi>),
            ),
        );
    }
}

#[derive(Resource)]
pub struct ShowInspector;

#[derive(Resource)]
pub struct PwmControl(bool);

#[derive(Resource)]
pub struct TimerUi(TimerState, TimerType);

pub enum TimerState {
    Running { start: Duration, offset: Duration },
    Paused { elapsed: Duration },
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum TimerType {
    Setup,
    Run,
    Cleanup,
}

#[derive(Component)]
pub struct MovementController;

#[derive(Component)]
pub struct MovementDebugger;

#[derive(Component)]
pub struct CurrentDrawDebugger;

#[derive(Component)]
pub struct PidHelper;

fn set_style(mut contexts: EguiContexts) {
    contexts.ctx_mut().set_visuals(if DARK_MODE {
        Visuals::dark()
    } else {
        Visuals::light()
    });
}

fn topbar(
    mut cmds: Commands,
    mut contexts: EguiContexts,

    robots: Query<
        (
            Entity,
            &Name,
            &Armed,
            Option<&DepthTarget>,
            Option<&OrientationTarget>,
        ),
        With<Robot>,
    >,

    cameras: Query<
        (Entity, &Name, Option<&VideoProcessorFactory>),
        (With<CameraDefinition>, With<VideoThread>),
    >,
    pipelines: Res<VideoPipelines>,

    inspector: Option<Res<ShowInspector>>,
    pwm_control: Option<Res<PwmControl>>,
    timer_ui: Option<Res<TimerUi>>,

    peers: Query<(&Peer, Option<&Name>)>,
    mut disconnect: EventWriter<DisconnectPeer>,
) {
    egui::TopBottomPanel::top("Top Bar").show(contexts.ctx_mut(), |ui| {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                ui.menu_button("Disconnect", |ui| {
                    if !peers.is_empty() {
                        for (peer, name) in &peers {
                            let text = if let Some(name) = name {
                                format!("{} ({})", name.as_str(), peer.token.0)
                            } else {
                                format!("{} ({})", peer.addrs, peer.token.0)
                            };

                            if ui.button(text).clicked() {
                                disconnect.send(DisconnectPeer(peer.token));
                            }
                        }
                    } else {
                        ui.label("No Connections");
                    }
                });

                if ui.button("Exit").clicked() {
                    cmds.queue(|world: &mut World| {
                        world.send_event(AppExit::Success);
                    })
                }
            });

            ui.menu_button("Sensors", |ui| {
                if ui.button("Calibrate Sea Level").clicked() {
                    cmds.queue(|world: &mut World| {
                        world.send_event(CalibrateSeaLevel);
                    })
                }

                if ui.button("Reset Servos").clicked() {
                    cmds.queue(|world: &mut World| {
                        world.send_event(ResetServos);
                    })
                }

                if ui.button("Reset Yaw").clicked() {
                    cmds.queue(|world: &mut World| {
                        world.send_event(ResetYaw);
                    })
                }
            });

            ui.menu_button("Cameras", |ui| {
                if ui.button("Resync Cameras").clicked() {
                    cmds.queue(|world: &mut World| {
                        world.send_event(ResyncCameras);
                    })
                }

                // TODO: Hide/Show All

                let cameras = cameras
                    .iter()
                    .map(|it| (it.1.as_str(), it))
                    .collect::<BTreeMap<_, _>>();

                for (entity, name, processor) in cameras.values() {
                    ui.menu_button(name.as_str(), |ui| {
                        // TODO: Hide/Show

                        let processor_name = processor.map(|it| &it.name);

                        for pipeline in &pipelines.0 {
                            let selected = processor_name == Some(&pipeline.name);
                            if ui
                                .selectable_label(selected, pipeline.name.as_str())
                                .clicked()
                            {
                                if !selected {
                                    cmds.entity(*entity).insert(pipeline.factory.clone());
                                } else {
                                    cmds.entity(*entity).remove::<VideoProcessorFactory>();
                                }
                            }
                        }
                    });
                }
            });

            ui.menu_button("View", |ui| {
                if ui
                    .selectable_label(inspector.is_some(), "ECS Inspector")
                    .clicked()
                {
                    if inspector.is_some() {
                        cmds.remove_resource::<ShowInspector>()
                    } else {
                        cmds.insert_resource(ShowInspector);
                    }
                }

                if ui.button("Movement Controller").clicked() {
                    cmds.spawn((
                        MovementController,
                        MovementContributionBundle {
                            name: Name::new("Manual Movement Controller"),
                            contribution: Default::default(),
                            robot: RobotId(NetId::invalid()),
                        },
                        Replicate,
                    ));
                }

                if ui.button("Movement Debugger").clicked() {
                    cmds.spawn((MovementDebugger, Replicate, RobotId(NetId::invalid())));
                }

                if ui.button("Current Draw Debugger").clicked() {
                    cmds.spawn((CurrentDrawDebugger, Replicate, RobotId(NetId::invalid())));
                }

                if ui.button("PID Helper").clicked() {
                    cmds.spawn((
                        PidData::default(),
                        PidHelper,
                        MovementContributionBundle {
                            name: Name::new("PID Helper"),
                            contribution: Default::default(),
                            robot: RobotId(NetId::invalid()),
                        },
                        Replicate,
                    ));
                }

                if ui
                    .selectable_label(pwm_control.is_some(), "PWM Control")
                    .clicked()
                {
                    if pwm_control.is_some() {
                        cmds.remove_resource::<PwmControl>()
                    } else {
                        cmds.insert_resource(PwmControl(false));
                    }
                }

                if ui.selectable_label(timer_ui.is_some(), "Timer").clicked() {
                    if timer_ui.is_some() {
                        cmds.remove_resource::<TimerUi>()
                    } else {
                        cmds.insert_resource(TimerUi(
                            TimerState::Paused {
                                elapsed: Duration::ZERO,
                            },
                            TimerType::Setup,
                        ));
                    }
                }

                if ui.button("Photo Sphere").clicked() {
                    for (robot, ..) in robots.iter() {
                        cmds.entity(robot).trigger(SpawnPhotoSphere);
                    }
                }
            });

            // RTL needs reverse order
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if !robots.is_empty() {
                    let mut layout_job = LayoutJob::default();

                    for (_entity, robot, state, depth_target, orientation_target) in &robots {
                        layout_job.append(
                            robot.as_str(),
                            20.0,
                            TextFormat {
                                color: if DARK_MODE {
                                    Color32::WHITE
                                } else {
                                    Color32::BLACK
                                },
                                ..default()
                            },
                        );
                        layout_job.append(
                            ":",
                            0.0,
                            TextFormat {
                                color: if DARK_MODE {
                                    Color32::WHITE
                                } else {
                                    Color32::BLACK
                                },
                                ..default()
                            },
                        );

                        // FIXME: Slight regression here since this the Armed status in the local
                        // esc could become out of sync with the robot's ecs
                        match state {
                            Armed::Disarmed => {
                                layout_job.append(
                                    "Disarmed",
                                    7.0,
                                    TextFormat {
                                        color: Color32::RED,
                                        ..default()
                                    },
                                );
                            }
                            Armed::Armed => {
                                layout_job.append(
                                    "Armed",
                                    7.0,
                                    TextFormat {
                                        color: Color32::GREEN,
                                        ..default()
                                    },
                                );

                                if let Some(&OrientationTarget(_)) = orientation_target {
                                    layout_job.append(
                                        "Orientation Hold",
                                        7.0,
                                        TextFormat {
                                            color: Color32::from_rgb(66, 145, 247),
                                            ..default()
                                        },
                                    );
                                }

                                if let Some(&DepthTarget(_)) = depth_target {
                                    layout_job.append(
                                        "Depth Hold",
                                        7.0,
                                        TextFormat {
                                            color: Color32::from_rgb(216, 123, 2),
                                            ..default()
                                        },
                                    );
                                }
                            }
                        };
                    }

                    ui.label(layout_job);
                } else {
                    ui.label(RichText::new(format!("No Robot")).color(if DARK_MODE {
                        Color32::WHITE
                    } else {
                        Color32::BLACK
                    }));
                }
            })
        });
    });
}

fn hud(
    mut cmds: Commands,

    mut host: Local<String>,
    runtime: ResMut<TokioTasksRuntime>,

    mut contexts: EguiContexts,
    attitude: Option<Res<OrientationDisplay>>,
    robots: Query<
        (
            &Name,
            Option<&Armed>,
            (Option<&MeasuredVoltage>, Option<&CurrentDraw>),
            (Option<&OrientationTarget>, Option<&TempertureMeasurement>),
            (
                Option<&SystemCpuTotal>,
                Option<&SystemLoadAverage>,
                Option<&SystemMemory>,
                Option<&SystemTemperatures>,
            ),
            (Option<&DepthMeasurement>, Option<&DepthTarget>),
            (Option<&Peer>, Option<&Latency>),
            &RobotId,
        ),
        With<Robot>,
    >,

    inputs: Query<
        (
            &SelectedServo,
            &InputInterpolation,
            &InputMap<Action>,
            &RobotId,
        ),
        With<InputMarker>,
    >,
    selected_camera: Query<(&Name, &RobotId), With<VideoMasterMarker>>,

    peers: Option<Res<MdnsPeers>>,

    mut disconnect: EventWriter<DisconnectPeer>,
) {
    let context = contexts.ctx_mut();

    // TODO(low): Support multiple robots
    if let Ok((
        robot_name,
        armed,
        (voltage, current_draw),
        (orientation_target, imu_temp),
        (cpu, load, memory, temps),
        (depth, depth_target),
        (peer, latency),
        robot_id,
    )) = robots.get_single()
    {
        let mut open = true;

        let window = egui::Window::new(robot_name.as_str())
            .id("HUD".into())
            .default_pos(context.screen_rect().right_top())
            .constrain_to(context.available_rect().shrink(20.0));
        // .movable(false);

        let window = if let Some(_peer) = peer {
            window.open(&mut open)
        } else {
            window
        };

        window.show(context, |ui| {
            let size = 20.0;

            ui.horizontal(|ui| {
                if let Some(attitude) = attitude {
                    ui.image(SizedTexture::new(attitude.1, (230.0, 230.0)));

                    ui.add_space(10.0);
                }

                ui.vertical(|ui| {
                    ui.allocate_space((230.0, 0.0).into());

                    if let Some(armed) = armed {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("Status:").size(size));
                            match armed {
                                Armed::Armed => {
                                    ui.label(
                                        RichText::new("Armed").size(size).color(Color32::GREEN),
                                    );
                                }
                                Armed::Disarmed => {
                                    ui.label(
                                        RichText::new("Disarmed").size(size).color(Color32::RED),
                                    );
                                }
                            }
                        });
                    }

                    if let Some((selected_servo, input_interpolation, input_map, _)) =
                        inputs.iter().find(|(_, _, _, robot)| **robot == *robot_id)
                    {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("Robot Mode:").size(size));
                            if *input_interpolation == InputInterpolation::normal() {
                                ui.label(RichText::new("Normal").size(size).color(Color32::GREEN));
                            } else if *input_interpolation == InputInterpolation::slow() {
                                ui.label(RichText::new("Slow").size(size).color(Color32::ORANGE));
                            } else if *input_interpolation == InputInterpolation::precision() {
                                ui.label(
                                    RichText::new("Precision").size(size).color(Color32::BLUE),
                                );
                            } else {
                                ui.label(RichText::new("Unknown").size(size).color(Color32::RED));
                            }
                        });

                        ui.add_space(10.0);

                        ui.horizontal(|ui| {
                            ui.label(RichText::new("Input Mode:").size(size));
                            if input_map.get(&Action::Pitch).is_some()
                                && input_map.get(&Action::Roll).is_some()
                            {
                                ui.label(
                                    RichText::new("Pitch & Roll")
                                        .size(size)
                                        .color(Color32::GOLD),
                                );
                            } else if input_map.get(&Action::Pitch).is_some() {
                                ui.label(RichText::new("Pitch").size(size).color(Color32::BLUE));
                            } else if input_map.get(&Action::Roll).is_some() {
                                ui.label(RichText::new("Roll").size(size).color(Color32::GREEN));
                            } else {
                                ui.label(RichText::new("Neither").size(size).color(Color32::RED));
                            }
                        });

                        ui.add_space(10.0);

                        ui.horizontal(|ui| {
                            ui.label(RichText::new("Servo:").size(size));
                            if let Some(selected_servo) = &selected_servo.servo {
                                ui.label(
                                    RichText::new(selected_servo.1.clone())
                                        .size(size)
                                        .color(Color32::GREEN),
                                );
                            } else {
                                ui.label(RichText::new("None").size(size).color(Color32::RED));
                            }
                        });
                    }

                    ui.add_space(10.0);

                    if let (Some(voltage), Some(current)) = (voltage, current_draw) {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("Power:").size(size));

                            let voltage_color;
                            if voltage.0 .0 < 11.5 {
                                voltage_color = Color32::RED;
                            } else if voltage.0 .0 < 12.5 {
                                voltage_color = Color32::YELLOW;
                            } else {
                                voltage_color = Color32::GREEN;
                            }

                            let current_color;
                            if current.0 .0 < 15.0 {
                                current_color = Color32::GREEN;
                            } else if current.0 .0 < 20.0 {
                                current_color = Color32::YELLOW;
                            } else {
                                current_color = Color32::RED;
                            }

                            ui.label(
                                RichText::new(format!("{}", voltage.0))
                                    .size(size)
                                    .color(voltage_color),
                            );
                            ui.label(
                                RichText::new(format!("{}", current.0))
                                    .size(size)
                                    .color(current_color),
                            );
                        });

                        ui.add_space(10.0);
                    }

                    if let Some(cpu) = cpu {
                        ui.label(RichText::new(format!("CPU: {:.2}%", cpu.0.usage)).size(size));
                    }
                    if let Some(load) = load {
                        ui.label(
                            RichText::new(format!(
                                "Load: {:.2}, {:.2}, {:.2}",
                                load.one_min, load.five_min, load.fifteen_min
                            ))
                            .size(size),
                        );
                    }

                    if let Some(memory) = memory {
                        let ram_usage = memory.used_mem as f64 / memory.total_mem as f64 * 100.0;
                        ui.label(RichText::new(format!("RAM: {:.2}%", ram_usage)).size(size));
                    }

                    if cpu.is_some() || load.is_some() || memory.is_some() {
                        ui.add_space(10.0);
                    }
                });

                ui.vertical(|ui| {
                    ui.allocate_space((230.0, 0.0).into());

                    if let (Some(peer), Some(latency)) = (peer, latency) {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("Address:").size(size));
                            ui.label(RichText::new(format!("{:?}", peer.addrs)).size(size * 0.75));
                        });

                        if let Some(ping) = latency.ping {
                            ui.label(
                                RichText::new(format!("Ping: {:.2?} frames", ping)).size(size),
                            );
                        }

                        ui.add_space(10.0);
                    }

                    if let Some(imu_temp) = imu_temp {
                        ui.label(
                            RichText::new(format!("IMU Temp: {}", imu_temp.temperature)).size(size),
                        );
                    }

                    if let Some(temps) = temps {
                        for temp in &temps.0 {
                            ui.label(
                                RichText::new(format!("{}: {}", temp.name, temp.tempature))
                                    .size(size),
                            );
                        }
                    }

                    // TODO: Find a way to support this again
                    // if let Some(depth) = depth {
                    //     ui.label(
                    //         RichText::new(format!("Water Temp: {}", depth.0.temperature))
                    //             .size(size),
                    //     );
                    // }

                    if imu_temp.is_some() || temps.is_some() {
                        ui.add_space(10.0);
                    }

                    if let Some(depth) = depth {
                        ui.label(RichText::new(format!("Depth: {}", depth.depth)).size(size));

                        if let Some(depth_target) = depth_target {
                            ui.label(
                                RichText::new(format!("Depth Target: {}", depth_target.0))
                                    .size(size),
                            );
                        }

                        ui.add_space(10.0);
                    }

                    if let Some(_orientation_target) = orientation_target {
                        ui.label(RichText::new("Orientation Control").size(size));
                    }

                    let selected_camera = selected_camera
                        .iter()
                        .filter(|(_, robot)| robot_id.0 == robot.0)
                        .map(|(it, _)| it.as_str())
                        .next();

                    if let Some(selected_camera) = selected_camera {
                        ui.label(RichText::new(format!("Camera: {selected_camera}")).size(size));
                    }
                });

                ui.allocate_space((0.0, 0.0).into());
            });
        });

        if let Some(peer) = peer {
            if !open {
                disconnect.send(DisconnectPeer(peer.token));
            }
        }
    } else {
        egui::Window::new("Not Connected")
            .id("HUD".into())
            .default_pos(context.screen_rect().right_top())
            .constrain_to(context.available_rect().shrink(20.0))
            // .movable(false)
            .show(contexts.ctx_mut(), |ui| {
                ui.horizontal(|ui| {
                    ui.label("Connect To:");
                    let line_response = ui.text_edit_singleline(&mut *host);
                    let button_response = ui.button("Connect");

                    if line_response.lost_focus() || button_response.clicked() {
                        let host = host.clone();
                        runtime.spawn_background_task(|mut ctx| async move {
                            let resolve = lookup_host(host).await;
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

                if let Some(peers) = peers {
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

                            ui.label(format!("{}@{}", name, host));

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
            });
    }
}

fn photosphere(
    mut cmds: Commands,
    mut contexts: EguiContexts,
    photospheres: Query<(Entity, &PhotoSphere)>,
) {
    for (entity, photosphere) in photospheres.iter() {
        let mut open = true;

        let context = contexts.ctx_mut();
        egui::Window::new("Photo Sphere")
            .id(Id::new(entity))
            .constrain_to(context.available_rect().shrink(20.0))
            .default_size((230.0, 230.0))
            .open(&mut open)
            .show(context, |ui| {
                let response = ui
                    .image(SizedTexture::new(
                        photosphere.view_texture_egui,
                        (ui.available_width(), ui.available_width()),
                    ))
                    .interact(Sense::DRAG);

                if response.dragged() {
                    info!("Dragged");
                    let delta = response.drag_delta();
                    cmds.entity(entity)
                        .trigger(RotatePhotoSphere(Vec2::new(delta.x, delta.y) / 100.0));
                }
                ui.image(SizedTexture::new(
                    photosphere.photo_sphere_egui,
                    (ui.available_width(), ui.available_width()),
                ));
            });

        if !open {
            cmds.entity(entity).despawn_recursive();
        }
    }
}

fn pwm_control(
    mut cmds: Commands,
    mut contexts: EguiContexts,
    mut pwm_control: ResMut<PwmControl>,
    robots: Query<(Entity, Option<&DisableMovementApi>, &RobotId), With<Robot>>,
    motors: Query<(
        Entity,
        Option<&MotorSignal>,
        Option<&MotorRawSignalRange>,
        &GenericMotorId,
        &RobotId,
    )>,
) {
    let mut open = true;

    egui::Window::new("PWM Control")
        // .current_pos(context.screen_rect().left_top())
        // .constrain_to(context.available_rect().shrink(20.0))
        .open(&mut open)
        .show(contexts.ctx_mut(), |ui| {
            if let Ok((robot, manual, robot_id)) = robots.get_single() {
                let mut enabled = pwm_control.0;
                ui.checkbox(&mut enabled, "Manual Enabled");

                if enabled != pwm_control.0 || enabled != manual.is_some() {
                    pwm_control.0 = enabled;

                    if enabled {
                        info!("Enabled manual control");
                        cmds.entity(robot).insert(DisableMovementApi);
                    } else {
                        info!("Disabled manual control");
                        cmds.entity(robot).remove::<DisableMovementApi>();
                    }
                }

                for (motor, signal, raw_range, channel, m_robot_id) in &motors {
                    if robot_id != m_robot_id {
                        continue;
                    }

                    let last_value = if let (Some(signal), Some(raw_range)) = (signal, raw_range) {
                        // This is repeated in s few places, mode into method on MotorSignal
                        match *signal {
                            MotorSignal::Percent(pct) => pct,
                            MotorSignal::Raw(raw) => raw_range.percent_from_raw(raw),
                        }
                    } else {
                        0.0
                    };
                    let mut value = last_value;

                    ui.horizontal(|ui| {
                        ui.label(format!("{}", channel.0));
                        ui.add(widgets::Slider::new(&mut value, -1.0..=1.0));
                        if ui.button("Clear").clicked() {
                            value = 0.0;
                        }
                    });

                    if value != last_value {
                        cmds.entity(motor).insert(MotorSignal::Percent(value));
                    }
                }
            } else {
                ui.label("No robot");
            };
        });

    if !open {
        cmds.remove_resource::<PwmControl>()
    }
}

fn cleanup_pwm_control(mut cmds: Commands, robots: Query<Entity, With<Robot>>) {
    info!("Disabled manual control");
    for robot in &robots {
        cmds.entity(robot).remove::<DisableMovementApi>();
    }
}

fn movement_control(
    mut cmds: Commands,
    mut contexts: EguiContexts,

    mut controllers: Query<
        (Entity, &mut RobotId, &mut MovementContribution),
        (With<MovementController>, Without<Robot>),
    >,
    robots: Query<(&Name, &RobotId, &MovementAxisMaximums), With<Robot>>,
    // motors: Query<(Entity, Option<&PwmSignal>, &PwmChannel, &RobotId)>,
) {
    for (contoller, mut selected_robot, mut contribution) in &mut controllers {
        let mut open = true;

        let context = contexts.ctx_mut();
        egui::Window::new("Movement Controller")
            .id(Id::new(contoller))
            .constrain_to(context.available_rect().shrink(20.0))
            .open(&mut open)
            .show(context, |ui| {
                ui.label("Robot:");
                let Some(maximums) = ui
                    .horizontal(|ui| {
                        let mut maximums = None;

                        for (name, robot_id, this_maximums) in &robots {
                            ui.selectable_value(&mut selected_robot.0, robot_id.0, name.as_str());

                            if selected_robot.0 == robot_id.0 {
                                maximums = Some(this_maximums.0.clone());
                            }
                        }
                        ui.selectable_value(&mut selected_robot.0, NetId::invalid(), "None");

                        if selected_robot.0 != NetId::invalid() {
                            maximums
                        } else {
                            None
                        }
                    })
                    .inner
                else {
                    return;
                };

                let mut movement = contribution.0;

                ui.horizontal(|ui| {
                    ui.add_sized([40.0, 0.0], Label::new("X:"));
                    let max = maximums[&Axis::X].0;
                    ui.add(widgets::Slider::new(&mut movement.force.x, -max..=max));
                });

                ui.horizontal(|ui| {
                    ui.add_sized([40.0, 0.0], Label::new("Y:"));
                    let max = maximums[&Axis::Y].0;
                    ui.add(widgets::Slider::new(&mut movement.force.y, -max..=max));
                });

                ui.horizontal(|ui| {
                    ui.add_sized([40.0, 0.0], Label::new("Z:"));
                    let max = maximums[&Axis::Z].0;
                    ui.add(widgets::Slider::new(&mut movement.force.z, -max..=max));
                });

                ui.horizontal(|ui| {
                    ui.add_sized([40.0, 0.0], Label::new("Pitch"));
                    let max = maximums[&Axis::XRot].0;
                    ui.add(widgets::Slider::new(&mut movement.torque.x, -max..=max));
                });

                ui.horizontal(|ui| {
                    ui.add_sized([40.0, 0.0], Label::new("Roll:"));
                    let max = maximums[&Axis::YRot].0;
                    ui.add(widgets::Slider::new(&mut movement.torque.y, -max..=max));
                });

                ui.horizontal(|ui| {
                    ui.add_sized([40.0, 0.0], Label::new("Yaw:"));
                    let max = maximums[&Axis::ZRot].0;
                    ui.add(widgets::Slider::new(&mut movement.torque.z, -max..=max));
                });

                ui.add_space(7.0);

                if ui.button("Clear").clicked() {
                    movement = MovementGlam::default();
                }

                if movement != contribution.0 {
                    contribution.0 = movement;
                }
            });

        if !open {
            cmds.entity(contoller).despawn();
        }
    }
}

fn movement_debug(
    mut cmds: Commands,
    mut contexts: EguiContexts,

    mut controllers: Query<(Entity, &mut RobotId), (With<MovementDebugger>)>,

    mut contributors: Query<(&Name, &MovementContribution, &RobotId), (Without<MovementDebugger>)>,
    robots: Query<
        (&Name, &RobotId, &TargetMovement, &ActualMovement),
        (With<Robot>, Without<MovementDebugger>),
    >,
) {
    for (contoller, mut selected_robot) in &mut controllers {
        let mut open = true;

        let context = contexts.ctx_mut();
        egui::Window::new("Movement Debugger")
            .id(Id::new(contoller))
            .constrain_to(context.available_rect().shrink(20.0))
            .open(&mut open)
            .show(context, |ui| {
                ui.label("Robot:");
                let Some((robot_id, target_movement, actual_movement)) = ui
                    .horizontal(|ui| {
                        let mut data = None;
                        for (name, robot_id, target_movement, actual_movement) in &robots {
                            ui.selectable_value(&mut selected_robot.0, robot_id.0, name.as_str());

                            if selected_robot.0 == robot_id.0 {
                                data = Some((robot_id, target_movement, actual_movement));
                            }
                        }
                        ui.selectable_value(&mut selected_robot.0, NetId::invalid(), "None");

                        if selected_robot.0 != NetId::invalid() {
                            data
                        } else {
                            None
                        }
                    })
                    .inner
                else {
                    return;
                };

                ui.label(format!("Target: {target_movement:.2?}"));
                ui.label(format!("Actual: {actual_movement:.2?}"));

                let mut movement = MovementGlam::default();

                for (name, contribution, other_robot_id) in contributors.iter() {
                    if robot_id != other_robot_id {
                        continue;
                    }

                    ui.label(format!("{}: {:.2?}", name.as_str(), contribution.0));
                    movement += contribution.0;
                }

                ui.label(format!(
                    "Unaccounted Movement: {:.2?}",
                    target_movement.0 - movement
                ));
            });

        if !open {
            cmds.entity(contoller).despawn();
        }
    }
}

fn current_draw_debug(
    mut cmds: Commands,
    mut contexts: EguiContexts,

    mut controllers: Query<(Entity, &mut RobotId), (With<CurrentDrawDebugger>)>,

    mut components: Query<
        (&Name, &CurrentDraw, &RobotId, Option<&ThrusterDefinition>),
        (Without<Robot>, Without<CurrentDrawDebugger>),
    >,
    robots: Query<
        (&Name, &RobotId, Option<&CurrentDraw>),
        (With<Robot>, Without<CurrentDrawDebugger>),
    >,
) {
    for (contoller, mut selected_robot) in &mut controllers {
        let mut open = true;

        let context = contexts.ctx_mut();
        egui::Window::new("Current Draw Debugger")
            .id(Id::new(contoller))
            .constrain_to(context.available_rect().shrink(20.0))
            .open(&mut open)
            .show(context, |ui| {
                ui.label("Robot:");
                let Some((robot_id, current_draw)) = ui
                    .horizontal(|ui| {
                        let mut data = None;
                        for (name, robot_id, current_draw) in &robots {
                            ui.selectable_value(&mut selected_robot.0, robot_id.0, name.as_str());

                            if selected_robot.0 == robot_id.0 {
                                data = Some((robot_id, current_draw));
                            }
                        }
                        ui.selectable_value(&mut selected_robot.0, NetId::invalid(), "None");

                        if selected_robot.0 != NetId::invalid() {
                            data
                        } else {
                            None
                        }
                    })
                    .inner
                else {
                    return;
                };

                if let Some(current_draw) = current_draw {
                    ui.label(format!("Actual Current Draw: {:.2?}", current_draw.0));
                }

                let mut current_draw_thrusters = Amperes::ZERO;
                let mut current_draw_other = Amperes::ZERO;

                for (name, current_draw, other_robot_id, thruster_definition) in components.iter() {
                    if robot_id != other_robot_id {
                        continue;
                    }

                    ui.label(format!("{}: {:.2?}", name.as_str(), current_draw.0));

                    if thruster_definition.is_some() {
                        current_draw_thrusters += current_draw.0;
                    } else {
                        current_draw_other += current_draw.0;
                    }
                }

                ui.label(format!(
                    "Thruster Current Draw: {:.2?}",
                    current_draw_thrusters
                ));
                ui.label(format!("Other Current Draw: {:.2?}", current_draw_other));

                let total_predicted = current_draw_thrusters + current_draw_other;
                ui.label(format!(
                    "Total Predicted Current Draw: {:.2?}",
                    total_predicted
                ));

                if let Some(current_draw) = current_draw {
                    ui.label(format!("Actual Current Draw: {:.2?}", current_draw.0));
                    ui.label(format!(
                        "Unaccounted Current Draw: {:.2?}",
                        current_draw.0 - total_predicted
                    ));
                }
            });

        if !open {
            cmds.entity(contoller).despawn();
        }
    }
}

#[derive(Component, Default)]
struct PidData {
    log: HashMap<PidAxis, PidDataEntry>,
    show_total: bool,
    show_error: bool,
    show_filtered_error: bool,
    show_kp: bool,
    show_ki: bool,
    show_kd: bool,
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, Copy)]
enum PidAxis {
    Yaw,
    Pitch,
    Roll,
    Depth,
}

struct PidDataEntry {
    error: VecDeque<PlotPoint>,
    filtered_error: VecDeque<PlotPoint>,
    total: VecDeque<PlotPoint>,
    kp: VecDeque<PlotPoint>,
    ki: VecDeque<PlotPoint>,
    kd: VecDeque<PlotPoint>,
}

impl Default for PidDataEntry {
    fn default() -> Self {
        Self {
            error: VecDeque::with_capacity(PID_SAMPLES + 5),
            filtered_error: VecDeque::with_capacity(PID_SAMPLES + 5),
            total: VecDeque::with_capacity(PID_SAMPLES + 5),
            kp: VecDeque::with_capacity(PID_SAMPLES + 5),
            ki: VecDeque::with_capacity(PID_SAMPLES + 5),
            kd: VecDeque::with_capacity(PID_SAMPLES + 5),
        }
    }
}

#[derive(Component)]
struct PidDisturbanceDeadline(Duration);

const PID_SAMPLES: usize = 500;
const PID_DISTURBANCE_TIME: Duration = Duration::from_millis(500);

// TODO: Use telemetry infra here after we get around to making that
fn pid_helper(
    mut cmds: Commands,
    mut contexts: EguiContexts,

    time: Res<Time<Real>>,

    mut controllers: Query<
        (
            Entity,
            &mut RobotId,
            &mut MovementContribution,
            &mut PidData,
            Option<&PidDisturbanceDeadline>,
        ),
        (With<PidHelper>, Without<Robot>),
    >,

    pid_controllers: Query<(&Name, &PidResult, &PidController, &RobotId), Without<PidData>>,

    robots: Query<(&Name, &RobotId, &MovementAxisMaximums), With<Robot>>,
    // motors: Query<(Entity, Option<&PwmSignal>, &PwmChannel, &RobotId)>,
) {
    for (controller, mut selected_robot, mut contribution, mut data, deadline) in &mut controllers {
        let mut open = true;

        let context = contexts.ctx_mut();
        egui::Window::new("Pid Helper")
            .id(Id::new(controller))
            .constrain_to(context.available_rect().shrink(20.0))
            .open(&mut open)
            .show(context, |ui| {
                ui.label("Robot:");
                let Some(maximums) = ui
                    .horizontal(|ui| {
                        let mut maximums = None;

                        for (name, robot_id, this_maximums) in &robots {
                            ui.selectable_value(&mut selected_robot.0, robot_id.0, name.as_str());

                            if selected_robot.0 == robot_id.0 {
                                maximums = Some(this_maximums.0.clone());
                            }
                        }
                        ui.selectable_value(&mut selected_robot.0, NetId::invalid(), "None");

                        if selected_robot.0 != NetId::invalid() {
                            maximums
                        } else {
                            None
                        }
                    })
                    .inner
                else {
                    return;
                };

                ui.toggle_value(&mut data.show_total, "Show Total");
                ui.toggle_value(&mut data.show_error, "Show Error");
                ui.toggle_value(&mut data.show_filtered_error, "Show Filtered");
                ui.toggle_value(&mut data.show_kp, "Show kp");
                ui.toggle_value(&mut data.show_ki, "Show ki");
                ui.toggle_value(&mut data.show_kd, "Show kd");

                ui.horizontal(|ui| {
                    let yaw = ui.selectable_label(data.log.contains_key(&PidAxis::Yaw), "Yaw");
                    if yaw.clicked() {
                        match data.log.entry(PidAxis::Yaw) {
                            Entry::Occupied(occupied_entry) => {
                                occupied_entry.remove();
                            }
                            Entry::Vacant(vacant_entry) => {
                                vacant_entry.insert(PidDataEntry::default());
                            }
                        }
                    }

                    let pitch =
                        ui.selectable_label(data.log.contains_key(&PidAxis::Pitch), "Pitch");
                    if pitch.clicked() {
                        match data.log.entry(PidAxis::Pitch) {
                            Entry::Occupied(occupied_entry) => {
                                occupied_entry.remove();
                            }
                            Entry::Vacant(vacant_entry) => {
                                vacant_entry.insert(PidDataEntry::default());
                            }
                        }
                    }

                    let roll = ui.selectable_label(data.log.contains_key(&PidAxis::Roll), "Roll");
                    if roll.clicked() {
                        match data.log.entry(PidAxis::Roll) {
                            Entry::Occupied(occupied_entry) => {
                                occupied_entry.remove();
                            }
                            Entry::Vacant(vacant_entry) => {
                                vacant_entry.insert(PidDataEntry::default());
                            }
                        }
                    }

                    let depth =
                        ui.selectable_label(data.log.contains_key(&PidAxis::Depth), "Depth");
                    if depth.clicked() {
                        match data.log.entry(PidAxis::Depth) {
                            Entry::Occupied(occupied_entry) => {
                                occupied_entry.remove();
                            }
                            Entry::Vacant(vacant_entry) => {
                                vacant_entry.insert(PidDataEntry::default());
                            }
                        }
                    }
                });

                for (axis, entry) in data.log.iter_mut() {
                    let controller_name = match axis {
                        PidAxis::Yaw => "Stabalize Yaw",
                        PidAxis::Pitch => "Stabalize Pitch",
                        PidAxis::Roll => "Stabalize Roll",
                        PidAxis::Depth => "Stabalize Depth",
                    };

                    let pid_result = pid_controllers.iter().find(|(name, _, _, robot_id)| {
                        **robot_id == *selected_robot && name.as_str() == controller_name
                    });
                    if let Some((_, pid_result, pid_controller, _)) = pid_result {
                        entry
                            .error
                            .push_back(PlotPoint::new(time.elapsed_secs_f64(), pid_result.error));
                        entry.filtered_error.push_back(PlotPoint::new(
                            time.elapsed_secs_f64(),
                            pid_controller.last_error(),
                        ));
                        entry.total.push_back(PlotPoint::new(
                            time.elapsed_secs_f64(),
                            pid_result.correction,
                        ));
                        entry
                            .kp
                            .push_back(PlotPoint::new(time.elapsed_secs_f64(), pid_result.p));
                        entry
                            .ki
                            .push_back(PlotPoint::new(time.elapsed_secs_f64(), pid_result.i));
                        entry
                            .kd
                            .push_back(PlotPoint::new(time.elapsed_secs_f64(), pid_result.d));

                        while entry.error.len() > PID_SAMPLES {
                            entry.error.pop_front();
                        }

                        while entry.filtered_error.len() > PID_SAMPLES {
                            entry.filtered_error.pop_front();
                        }

                        while entry.total.len() > PID_SAMPLES {
                            entry.total.pop_front();
                        }

                        while entry.kp.len() > PID_SAMPLES {
                            entry.kp.pop_front();
                        }

                        while entry.ki.len() > PID_SAMPLES {
                            entry.ki.pop_front();
                        }

                        while entry.kd.len() > PID_SAMPLES {
                            entry.kd.pop_front();
                        }
                    }
                }

                for (axis, entry) in data.log.iter() {
                    ui.label(format!("{axis:?} Plot"));
                    Plot::new(format!("Pid Tuning Plot {axis:?}"))
                        .height(300.0)
                        .show(ui, |plot| {
                            if data.show_error {
                                let (first, second) = entry.error.as_slices();
                                plot.add(
                                    Line::new(format!("{axis:?}, error"), first)
                                        .stroke((1.5, Color32::BROWN)),
                                );
                                plot.add(
                                    Line::new(format!("{axis:?}, error"), second)
                                        .stroke((1.5, Color32::BROWN)),
                                );
                            }

                            if data.show_filtered_error {
                                let (first, second) = entry.filtered_error.as_slices();
                                plot.add(
                                    Line::new(format!("{axis:?}, filtered error"), first)
                                        .stroke((1.5, Color32::BROWN)),
                                );
                                plot.add(
                                    Line::new(format!("{axis:?}, filtered error"), second)
                                        .stroke((1.5, Color32::BROWN)),
                                );
                            }

                            if data.show_total {
                                let (first, second) = entry.total.as_slices();
                                plot.add(
                                    Line::new(format!("{axis:?}, total"), first)
                                        .stroke((1.5, Color32::BLACK)),
                                );
                                plot.add(
                                    Line::new(format!("{axis:?}, total"), second)
                                        .stroke((1.5, Color32::BLACK)),
                                );
                            }

                            if data.show_kp {
                                let (first, second) = entry.kp.as_slices();
                                plot.add(
                                    Line::new(format!("{axis:?}, kp"), first)
                                        .stroke((1.5, Color32::RED)),
                                );
                                plot.add(
                                    Line::new(format!("{axis:?}, kp"), second)
                                        .stroke((1.5, Color32::RED)),
                                );
                            }

                            if data.show_ki {
                                let (first, second) = entry.ki.as_slices();
                                plot.add(
                                    Line::new(format!("{axis:?}, ki"), first)
                                        .stroke((1.5, Color32::GREEN)),
                                );
                                plot.add(
                                    Line::new(format!("{axis:?}, ki"), second)
                                        .stroke((1.5, Color32::GREEN)),
                                );
                            }

                            if data.show_kd {
                                let (first, second) = entry.kd.as_slices();
                                plot.add(
                                    Line::new(format!("{axis:?}, kd"), first)
                                        .stroke((1.5, Color32::BLUE)),
                                );
                                plot.add(
                                    Line::new(format!("{axis:?}, kd"), second)
                                        .stroke((1.5, Color32::BLUE)),
                                );
                            }
                        });

                    ui.add_space(7.0);
                }

                let mut movement = contribution.0;

                if let Some(deadline) = deadline {
                    if time.elapsed() > deadline.0 || ui.button("Clear").clicked() {
                        movement = MovementGlam::default();
                        cmds.entity(controller).remove::<PidDisturbanceDeadline>();
                    }
                } else {
                    movement = MovementGlam::default();
                }

                if ui.button("Yaw Disturbance").clicked() {
                    movement = MovementGlam {
                        force: vec3a(0.0, 0.0, 0.0),
                        torque: vec3a(0.0, 0.0, 10.0),
                    };
                    cmds.entity(controller).insert(PidDisturbanceDeadline(
                        time.elapsed() + PID_DISTURBANCE_TIME,
                    ));
                }

                if ui.button("Pitch Disturbance").clicked() {
                    movement = MovementGlam {
                        force: vec3a(0.0, 0.0, 0.0),
                        torque: vec3a(10.0, 0.0, 0.0),
                    };
                    cmds.entity(controller).insert(PidDisturbanceDeadline(
                        time.elapsed() + PID_DISTURBANCE_TIME,
                    ));
                }

                if ui.button("Roll Disturbance").clicked() {
                    movement = MovementGlam {
                        force: vec3a(0.0, 0.0, 0.0),
                        torque: vec3a(0.0, 10.0, 0.0),
                    };
                    cmds.entity(controller).insert(PidDisturbanceDeadline(
                        time.elapsed() + PID_DISTURBANCE_TIME,
                    ));
                }

                if ui.button("Depth Disturbance").clicked() {
                    movement = MovementGlam {
                        force: vec3a(0.0, 0.0, -10.0),
                        torque: vec3a(0.0, 0.0, 0.0),
                    };
                    cmds.entity(controller).insert(PidDisturbanceDeadline(
                        time.elapsed() + PID_DISTURBANCE_TIME,
                    ));
                }

                ui.add_space(7.0);

                if movement != contribution.0 {
                    contribution.0 = movement;
                }
            });

        if !open {
            cmds.entity(controller).despawn();
        }
    }
}

fn timer(
    mut cmds: Commands,
    mut contexts: EguiContexts,
    mut timer: ResMut<TimerUi>,
    time: Res<Time<Real>>,
) {
    let context = contexts.ctx_mut();
    let mut open = true;

    egui::Window::new("Timer")
        .default_pos(context.screen_rect().left_top())
        .constrain_to(context.available_rect().shrink(20.0))
        .open(&mut open)
        .show(contexts.ctx_mut(), |ui| {
            let current_value = &mut timer.1;
            ui.horizontal(|ui| {
                ui.selectable_value(current_value, TimerType::Setup, "Setup");
                ui.selectable_value(current_value, TimerType::Run, "Demo");
                ui.selectable_value(current_value, TimerType::Cleanup, "Cleanup");
            });

            let total_duration = match current_value {
                TimerType::Setup => Duration::from_secs_f64(5.0 * 60.0),
                TimerType::Run => Duration::from_secs_f64(15.0 * 60.0),
                TimerType::Cleanup => Duration::from_secs_f64(5.0 * 60.0),
            };

            let remaining_duration = match timer.0 {
                TimerState::Running { start, offset } => {
                    total_duration.saturating_sub((time.elapsed() - start) + offset)
                }
                TimerState::Paused { elapsed } => total_duration - elapsed,
            };

            let remaining_sec = remaining_duration.as_secs();

            let min = remaining_sec / 60;
            let sec = remaining_sec % 60;

            ui.allocate_ui((ui.available_width(), 25.0).into(), |ui| {
                ui.centered_and_justified(|ui| {
                    ui.label(RichText::new(format!("{min:02}:{sec:02}",)).size(20.0));
                });
            });
            ui.horizontal(|ui| match timer.0 {
                TimerState::Running { start, offset } => {
                    if ui.button("Pause").clicked() {
                        timer.0 = TimerState::Paused {
                            elapsed: time.elapsed() - start + offset,
                        };
                    }
                    if ui.button("Reset").clicked() {
                        timer.0 = TimerState::Paused {
                            elapsed: Duration::ZERO,
                        };
                    }
                }
                TimerState::Paused { elapsed } => {
                    if ui.button("Resume").clicked() {
                        timer.0 = TimerState::Running {
                            start: time.elapsed(),
                            offset: elapsed,
                        };
                    }
                    if ui.button("Reset").clicked() {
                        timer.0 = TimerState::Paused {
                            elapsed: Duration::ZERO,
                        };
                    }
                }
            });
        });

    if !open {
        cmds.remove_resource::<TimerUi>();
    }
}
