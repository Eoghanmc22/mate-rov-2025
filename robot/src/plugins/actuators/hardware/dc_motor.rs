use std::{
    mem,
    sync::Arc,
    time::{Duration, Instant},
};

use ahash::HashMap;
use anyhow::{anyhow, Context};
use bevy::{app::AppExit, prelude::*};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::{
    components::{Armed, CurrentDraw, GenericMotorId, MotorRawSignalRange, MotorSignal, RobotId},
    ecs_sync::NetId,
    error::{self, Errors},
    types::units::Amperes,
};
// use crossbeam::channel::{self, Sender};
use dc_motor_interface::{
    c2h::{self, PacketC2H},
    h2c::{self, PacketH2C},
    implementation_tokio::{DcMotorController, DcMotorControllerHandle},
    Interval, Motors, Speed,
};
use tokio::{
    select,
    sync::{
        broadcast,
        mpsc::{self, Sender},
        Notify,
    },
    time,
};

use crate::plugins::core::robot::{LocalRobot, LocalRobotMarker};

use super::motor_id_map::{DcChannel, LocalMotorId};

pub struct DcMotorPlugin;

impl Plugin for DcMotorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, start_dc_motor_thread.pipe(error::handle_errors));
        app.add_systems(
            PostUpdate,
            listen_to_dc_motors
                .pipe(error::handle_errors)
                .run_if(resource_exists::<DcMotorChannels>),
        );
        app.add_systems(Last, shutdown.run_if(resource_exists::<DcMotorChannels>));
    }
}

#[derive(Resource)]
struct DcMotorChannels(Sender<DcMotorEvent>);

#[derive(Debug)]
enum DcMotorEvent {
    Arm(Armed),
    UpdateChannel(DcChannel, i16),
    BatchComplete,
    Shutdown,
}

// TODO:
// - Impl Read software data packer
// - Impl support for flashing motor controller when there is a version mismatch
// - This may not be robust against the usb link droping out
// - Figure out how to use tracing spans in the async tasks
fn start_dc_motor_thread(
    mut cmds: Commands,
    runtime: ResMut<TokioTasksRuntime>,
    local_robot: Res<LocalRobot>,
    errors: Res<Errors>,
) -> anyhow::Result<()> {
    let interval = Duration::from_secs_f32(1.0 / 100.0);
    let max_inactive = Duration::from_secs_f32(1.0 / 10.0);

    let ping_interval = Duration::from_secs_f32(1.0 / 25.0);
    let max_ping_latency = Duration::from_millis(10);

    let motor_controller = DcMotorController::open(DcMotorControllerHandle::FirstAvaible)
        .context("Get motor controller interface")?;

    let (tx_data, mut rx_data) = mpsc::channel(10);

    const STOP_SIGNALS: [i16; 4] = [0; 4];

    cmds.insert_resource(DcMotorChannels(tx_data));

    let errors = errors.0.clone();
    let (tx_out, rx_out) = mpsc::channel(10);
    let (tx_in, mut rx_in) = broadcast::channel(10);
    let connected = Arc::new(Notify::new());

    // Telemetry read back task
    runtime.spawn_background_task({
        let errors = errors.clone();
        let mut rx_in = tx_in.subscribe();
        let local_robot = local_robot.net_id;

        async move |mut ctx| -> anyhow::Result<()> {
            loop {
                match rx_in.recv().await? {
                    PacketC2H::MotorState(state) => {
                        ctx.run_on_main_thread(move |ctx| {
                            // TODO: Replace with a filtered query when we add a marker component to local
                            // entities
                            let mut query =
                                ctx.world.query::<(Entity, &GenericMotorId, &RobotId)>();
                            let Some((entity, ..)) =
                                query.iter(ctx.world).find(|(_, &motor, robot)| {
                                    robot.0 == local_robot
                                        && matches!(motor.into(), LocalMotorId::DcChannel(ch)
                                if ch.id() == state.motor_id)
                                })
                            else {
                                return;
                            };

                            // TODO: Also put fault status in world
                            ctx.world
                                .entity_mut(entity)
                                .insert(CurrentDraw(Amperes(state.current_draw.as_f32_amps())));
                        })
                        .await;
                    }
                    PacketC2H::Error(err) => {
                        let _ =
                            errors.send(anyhow!("DC Motor controller reported an error: {err:?}"));
                    }
                    _ => {}
                }
            }
        }
    });

    // Ping task
    runtime.spawn_background_task({
        let connected = connected.clone();
        let tx_out = tx_out.clone();
        let mut rx_in = tx_in.subscribe();

        async move |_| -> anyhow::Result<()> {
            connected.notified().await;

            let mut interval = time::interval(ping_interval);
            let mut tx_id = 100;
            let mut un_acked_pings = 0;

            loop {
                interval.tick().await;

                tx_out.send(h2c::Ping { id: tx_id }.into()).await?;
                tx_id += 1;

                let deadline = Instant::now() + max_ping_latency;

                let acked = loop {
                    if Instant::now() > deadline {
                        break false;
                    }

                    let Ok(Ok(PacketC2H::Pong(c2h::Pong { id: rx_id }))) =
                        time::timeout(max_ping_latency, rx_in.recv()).await
                    else {
                        assert!(!rx_in.is_closed());
                        continue;
                    };

                    break tx_id == rx_id;
                };

                if !acked {
                    warn!("DC Motor controller did not ack ping ({un_acked_pings})");
                    un_acked_pings += 1;
                } else {
                    un_acked_pings = 0;
                }

                // TODO: explode if un_acked_pings passes a threshold
            }
        }
    });

    // Signal output and setup task
    runtime.spawn_background_task(async move |_| -> anyhow::Result<()> {
        // let _span = span!(Level::INFO, "Motor Controller Bridge").entered();

        loop {
            tx_out.send(PacketH2C::ReadProtocolVersion).await?;
            if let PacketC2H::ProtocolVersionResponse(version) = rx_in.recv().await? {
                assert!(version.version == dc_motor_interface::PROTOCOL_VERSION);
                break;
            }
        }

        tx_out.send(h2c::SetArmed::Disarmed.into()).await?;
        tx_out
            .send(
                h2c::StartStream {
                    motors: Motors::all(),
                    interval: Interval::from_duration(interval),
                }
                .into(),
            )
            .await?;
        tx_out
            .send(
                h2c::SetSpeed {
                    motors: Motors::all(),
                    speed: Speed(0),
                }
                .into(),
            )
            .await?;

        info!("DC Motor Controller bridge thread starting");
        connected.notify_waiters();

        let mut next_channel_pwms = HashMap::default();
        let mut batch_started = false;

        let mut last_armed = Armed::Disarmed;
        let mut armed = Armed::Disarmed;
        let mut channel_pwms = HashMap::default();
        let mut last_batch = Instant::now();

        let mut do_shutdown = false;
        let mut interval = time::interval(interval);

        while !do_shutdown {
            interval.tick().await;

            while let Ok(event) = rx_data.try_recv() {
                trace!(?event, "Got DcMotorEvent");

                match event {
                    DcMotorEvent::Arm(Armed::Armed) => {
                        batch_started = true;
                        next_channel_pwms.clear();
                    }
                    DcMotorEvent::Arm(Armed::Disarmed) => {
                        batch_started = false;
                        armed = Armed::Disarmed;
                    }
                    DcMotorEvent::UpdateChannel(channel, pwm) => {
                        if batch_started {
                            next_channel_pwms.insert(channel, pwm);
                        }
                    }
                    DcMotorEvent::BatchComplete => {
                        if batch_started {
                            batch_started = false;

                            armed = Armed::Armed;
                            channel_pwms = mem::take(&mut next_channel_pwms);
                            last_batch = Instant::now();
                        }
                    }
                    DcMotorEvent::Shutdown => {
                        armed = Armed::Disarmed;
                        do_shutdown = true;

                        break;
                    }
                }
            }

            assert!(!rx_data.is_closed());
            // Update state
            if matches!(armed, Armed::Armed) && last_batch.elapsed() > max_inactive {
                warn!("Time since last batch exceeded max_inactive, disarming");

                // TODO(mid): Should this notify bevy?
                let _ = errors.send(anyhow!("Motors disarmed due to inactivity"));
                armed = Armed::Disarmed;
            }

            // Sync state with pwm chip
            match armed {
                Armed::Armed => {
                    let res = tx_out
                        .send(
                            h2c::SetArmed::Armed {
                                duration: Interval::from_duration(max_inactive),
                            }
                            .into(),
                        )
                        .await;

                    if let Err(err) = res {
                        let _ = errors.send(
                            anyhow::format_err!(err).context("Dc Motor interface tx channel error"),
                        );
                    }
                }
                Armed::Disarmed => {
                    let res = tx_out.send(h2c::SetArmed::Disarmed.into()).await;

                    // No motors should be active when disarmed
                    channel_pwms.clear();

                    if let Err(err) = res {
                        let _ = errors.send(
                            anyhow::format_err!(err).context("Dc Motor interface tx channel error"),
                        );
                    }
                }
            }

            // Generate the pwm states for each channel
            let pwms = {
                // By default all motors should be stopped
                let mut pwms = STOP_SIGNALS;

                // Copy pwm values from `channel_pwms` into `pwms`
                // `channel_pwms` is cleared in the disarmed case
                for (channel, new_pwm) in &channel_pwms {
                    let channel_pwm = pwms.get_mut(channel.id() as usize);

                    // If this is a valid channel, set the corresponding channel's pwm
                    if let Some(channel_pwm) = channel_pwm {
                        *channel_pwm = *new_pwm;
                    }
                }

                pwms
            };

            trace!(?armed, ?channel_pwms, ?pwms, "Writing Pwms");

            // Write the current pwms to the pwm chip
            for (idx, pwm) in pwms.iter().enumerate() {
                let res = tx_out
                    .send(
                        h2c::SetSpeed {
                            motors: Motors::from_bits_truncate(1u8 << idx),
                            speed: Speed(*pwm),
                        }
                        .into(),
                    )
                    .await;

                if let Err(err) = res {
                    let _ = errors.send(
                        anyhow::format_err!(err).context("Dc Motor interface tx channel error"),
                    );
                }
            }

            if last_armed != armed {
                info!("DC Motor Controller: {armed:?}");

                last_armed = armed;
            }
        }

        warn!("DC Motor Controller bridge thread died");

        Ok(())
    });

    runtime.spawn_background_task(async move |_| {
        // let _span = span!(Level::INFO, "Motor Controller Serial").entered();

        motor_controller.start(tx_in, rx_out).await;

        warn!("DC Motor Controller interface thread died");
    });
    // .context("Spawn thread")?;

    Ok(())
}

fn listen_to_dc_motors(
    channels: Res<DcMotorChannels>,
    robot: Query<(&NetId, &Armed), With<LocalRobotMarker>>,
    pwms: Query<(
        &RobotId,
        &GenericMotorId,
        &MotorSignal,
        &MotorRawSignalRange,
    )>,
) -> anyhow::Result<()> {
    let (net_id, armed) = robot.single();

    channels
        .0
        .blocking_send(DcMotorEvent::Arm(*armed))
        .context("Send data to dc motor thread")?;

    for (RobotId(robot_net_id), &channel, &signal, raw_range) in &pwms {
        if robot_net_id != net_id {
            continue;
        }

        let LocalMotorId::DcChannel(channel) = channel.into() else {
            continue;
        };

        let output = match signal {
            MotorSignal::Percent(pct) => raw_range.raw_from_percent(pct),
            MotorSignal::Raw(raw) => raw,
        };
        let output = raw_range.clamp_raw(output) as i16;

        channels
            .0
            .blocking_send(DcMotorEvent::UpdateChannel(channel, output))
            .context("Send data to dc motor thread")?;
    }

    channels
        .0
        .blocking_send(DcMotorEvent::BatchComplete)
        .context("Send data to dc motor thread")?;

    Ok(())
}

fn shutdown(channels: Res<DcMotorChannels>, mut exit: EventReader<AppExit>) {
    for _event in exit.read() {
        let _ = channels.0.send(DcMotorEvent::Shutdown);
    }
}
