use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context};
use bevy::{app::AppExit, prelude::*};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::{
    components::{Armed, CurrentDraw, GenericMotorId, MotorRawSignalRange, MotorSignal, RobotId},
    ecs_sync::NetId,
    error::{self, Errors},
    types::units::Amperes,
};
use dc_motor_interface::{
    c2h::{self, MotorState, PacketC2H},
    h2c::{self, PacketH2C},
    implementation_tokio::{DcMotorController, DcMotorControllerHandle},
    Interval, Motors, Speed,
};
use tokio::{
    sync::{
        broadcast::{self, error::RecvError},
        mpsc::{self, Receiver, Sender},
        Notify,
    },
    time,
};

use super::motor_id_map::{DcChannel, LocalMotorId};
use crate::plugins::core::robot::{LocalRobot, LocalRobotMarker};

const NUM_CHANNELS: usize = 4;
// fraction of output
type ChannelBatch = [i16; NUM_CHANNELS];
const STOP_SIGNALS: ChannelBatch = [0; NUM_CHANNELS];

pub struct DcMotorPlugin;

impl Plugin for DcMotorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, start_dc_motor_thread.pipe(error::handle_errors));
        app.add_systems(
            PreUpdate,
            read_telemetry.run_if(resource_exists::<DcMotorChannels>),
        );
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
struct DcMotorChannels(Sender<DcMotorEvent>, Receiver<MotorState>);

#[derive(Debug)]
enum DcMotorEvent {
    Arm(Armed),
    Batch(ChannelBatch),
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
    errors: Res<Errors>,
) -> anyhow::Result<()> {
    let interval = Duration::from_secs_f32(1.0 / 100.0);
    let max_inactive = Duration::from_secs_f32(1.0 / 10.0);

    let ping_interval = Duration::from_secs_f32(1.0 / 25.0);
    let max_ping_latency = Duration::from_millis(500);

    let (tx_data, mut rx_data) = mpsc::channel(10);
    let (tx_state, rx_state) = mpsc::channel(10);

    cmds.insert_resource(DcMotorChannels(tx_data, rx_state));

    let errors = errors.0.clone();
    let (tx_out, rx_out) = mpsc::channel(10);
    let (tx_in, mut rx_in) = broadcast::channel(10);
    let connected = Arc::new(Notify::new());

    // Telemetry read back task
    runtime.spawn_background_task({
        let errors = errors.clone();
        let mut rx_in = tx_in.subscribe();

        async move |_| loop {
            match rx_in.recv().await {
                Ok(PacketC2H::MotorState(state)) => {
                    let res = tx_state.send(state).await;
                    res.unwrap();
                }
                Ok(PacketC2H::Error(err)) => {
                    let _ = errors.send(anyhow!("DC Motor controller reported an error: {err:?}"));
                }
                Ok(_) => {}
                Err(RecvError::Lagged(count)) => {
                    warn!("Telemetry dc rx lagged: {count}");
                }
                Err(RecvError::Closed) => {
                    warn!("Telemetry dc rx closed");
                    return;
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

                tx_id += 1;

                // TODO: explode if un_acked_pings passes a threshold
            }
        }
    });

    // Signal output and setup task
    runtime.spawn_background_task({
        let errors = errors.clone();

        async move |_| -> anyhow::Result<()> {
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

            let mut last_armed = Armed::Disarmed;
            let mut armed = Armed::Disarmed;
            let mut channel_signals = STOP_SIGNALS;
            let mut last_arm_timestamp = Instant::now();

            let mut do_shutdown = false;
            let mut interval = time::interval(interval);

            while !do_shutdown {
                interval.tick().await;

                while let Ok(event) = rx_data.try_recv() {
                    trace!(?event, "Got DcMotorEvent");

                    match event {
                        DcMotorEvent::Arm(Armed::Armed) => {
                            armed = Armed::Armed;
                            last_arm_timestamp = Instant::now();
                        }
                        DcMotorEvent::Arm(Armed::Disarmed) => {
                            armed = Armed::Disarmed;
                            channel_signals = STOP_SIGNALS;
                        }
                        DcMotorEvent::Batch(new_channel_signals) => {
                            if armed == Armed::Armed {
                                channel_signals = new_channel_signals;
                            } else {
                                channel_signals = STOP_SIGNALS;
                            }
                        }
                        DcMotorEvent::Shutdown => {
                            armed = Armed::Disarmed;
                            channel_signals = STOP_SIGNALS;
                            do_shutdown = true;

                            break;
                        }
                    }
                }
                if rx_data.is_closed() {
                    do_shutdown = true;
                }

                // Update state
                if matches!(armed, Armed::Armed) && last_arm_timestamp.elapsed() > max_inactive {
                    warn!("Time since last arm exceeded max_inactive, disarming");

                    let _ = errors.send(anyhow!("Motors disarmed due to inactivity"));
                    armed = Armed::Disarmed;
                    channel_signals = STOP_SIGNALS;
                }

                // Sync state with pwm chip
                let res = match armed {
                    Armed::Armed => {
                        tx_out
                            .send(
                                h2c::SetArmed::Armed {
                                    duration: Interval::from_duration(max_inactive),
                                }
                                .into(),
                            )
                            .await
                    }
                    Armed::Disarmed => {
                        channel_signals = STOP_SIGNALS;
                        tx_out.send(h2c::SetArmed::Disarmed.into()).await
                    }
                };

                if let Err(err) = res {
                    let _ = errors.send(
                        anyhow::format_err!(err).context("Dc Motor interface tx channel error"),
                    );
                }

                trace!(?armed, ?channel_signals, "Writing Signals");

                // Write the current pwms to the pwm chip
                for (idx, pwm) in channel_signals.iter().enumerate() {
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
        }
    });

    runtime.spawn_background_task(async move |_| {
        // let _span = span!(Level::INFO, "Motor Controller Serial").entered();

        let motor_controller = match DcMotorController::open(DcMotorControllerHandle::FirstAvaible)
            .context("Get motor controller interface")
        {
            Ok(motor_controller) => motor_controller,
            Err(err) => {
                let _ = errors
                    .send(anyhow::format_err!(err).context("Dc Motor interface tx channel error"));
                return;
            }
        };

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

    let mut channel_batch = STOP_SIGNALS;
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

        let id = channel.id() as usize;
        if id < NUM_CHANNELS {
            channel_batch[id] = output;
        } else {
            warn!("Attempted to drive unknown dc channel {id}");
        }
    }

    channels
        .0
        .blocking_send(DcMotorEvent::Batch(channel_batch))
        .context("Send data to dc motor thread")?;

    Ok(())
}

fn read_telemetry(
    mut cmds: Commands,
    mut channels: ResMut<DcMotorChannels>,
    local_robot: Res<LocalRobot>,
    query: Query<(Entity, &GenericMotorId, &RobotId)>,
) {
    while let Ok(state) = channels.1.try_recv() {
        let Some((entity, ..)) = query.iter().find(|(_, &motor, robot)| {
            robot.0 == local_robot.net_id
                && matches!(motor.into(), LocalMotorId::DcChannel(ch)
                                if ch.id() == state.motor_id)
        }) else {
            return;
        };

        // TODO: Also put fault status in world
        cmds.entity(entity)
            .insert(CurrentDraw(Amperes(state.current_draw.as_f32_amps())));
    }
}

fn shutdown(channels: Res<DcMotorChannels>, mut exit: EventReader<AppExit>) {
    for _event in exit.read() {
        let _ = channels.0.send(DcMotorEvent::Shutdown);
    }
}
