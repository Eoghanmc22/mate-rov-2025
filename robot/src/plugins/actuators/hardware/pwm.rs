use std::{
    array, thread,
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context};
use bevy::{app::AppExit, prelude::*};
use common::{
    components::{Armed, GenericMotorId, MotorRawSignalRange, MotorSignal, RobotId},
    ecs_sync::NetId,
    error::{self, Errors},
};
use crossbeam::channel::{self, Sender};
use tracing::{span, Level};

use super::motor_id_map::LocalMotorId;
use crate::{peripheral::pca9685::Pca9685, plugins::core::robot::LocalRobotMarker};

const NUM_CHANNELS: usize = 16;
// microseconds
type ChannelBatch = [u16; NUM_CHANNELS];
type ChannelPwms = [Duration; NUM_CHANNELS];
const STOP_SIGNALS: ChannelBatch = [1500; NUM_CHANNELS];
const STOP_PWMS: ChannelPwms = [Duration::from_micros(1500); NUM_CHANNELS];

pub struct PwmOutputPlugin;

impl Plugin for PwmOutputPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, start_pwm_thread.pipe(error::handle_errors));
        app.add_systems(
            PostUpdate,
            listen_to_pwms
                .pipe(error::handle_errors)
                .run_if(resource_exists::<GenericMotorIds>),
        );
        app.add_systems(Last, shutdown.run_if(resource_exists::<GenericMotorIds>));
    }
}

#[derive(Resource)]
struct GenericMotorIds(Sender<PwmEvent>);

#[derive(Debug)]
enum PwmEvent {
    Arm(Armed),
    Batch(ChannelBatch),
    Shutdown,
}

fn start_pwm_thread(mut cmds: Commands, errors: Res<Errors>) -> anyhow::Result<()> {
    let interval = Duration::from_secs_f32(1.0 / 100.0);
    let max_inactive = Duration::from_secs_f32(1.0 / 10.0);
    let arming_duration = Duration::from_millis(1500);

    let (tx_data, rx_data) = channel::bounded(30);

    let mut pwm_controller =
        Pca9685::new(Pca9685::I2C_BUS, Pca9685::I2C_ADDRESS, interval).context("PCA9685")?;

    pwm_controller
        .set_pwms(STOP_PWMS)
        .context("Set initial pwms")?;

    pwm_controller.output_disable();

    cmds.insert_resource(GenericMotorIds(tx_data));

    let errors = errors.0.clone();
    thread::Builder::new()
        .name("PWM Thread".to_owned())
        .spawn(move || {
            let _span = span!(Level::INFO, "Pwm Output Thread").entered();

            let mut deadline = Instant::now();

            let mut last_armed = Armed::Disarmed;
            let mut armed = Armed::Disarmed;
            let mut channel_pwms = STOP_PWMS;
            let mut last_arm_timestamp = Instant::now();
            let mut last_rearm_timestamp = Instant::now();

            let mut do_shutdown = false;

            while !do_shutdown {
                let span = span!(Level::INFO, "Pwm Output Cycle").entered();

                // Process events
                for event in rx_data.try_iter() {
                    trace!(?event, "Got PwmEvent");

                    match event {
                        PwmEvent::Arm(Armed::Armed) => {
                            last_arm_timestamp = Instant::now();
                            if armed != Armed::Armed {
                                last_rearm_timestamp = last_arm_timestamp;
                            }
                            armed = Armed::Armed;
                        }
                        PwmEvent::Arm(Armed::Disarmed) => {
                            armed = Armed::Disarmed;
                            channel_pwms = STOP_PWMS;
                        }
                        PwmEvent::Batch(new_channel_signals) => {
                            if armed == Armed::Armed {
                                channel_pwms = array::from_fn(|idx| {
                                    Duration::from_micros(new_channel_signals[idx] as u64)
                                })
                            } else {
                                channel_pwms = STOP_PWMS;
                            }
                        }
                        PwmEvent::Shutdown => {
                            armed = Armed::Disarmed;
                            channel_pwms = STOP_PWMS;
                            do_shutdown = true;

                            break;
                        }
                    }
                }

                // Update state
                if matches!(armed, Armed::Armed) && last_arm_timestamp.elapsed() > max_inactive {
                    warn!("Time since last arm exceeded max_inactive, disarming");

                    let _ = errors.send(anyhow!("Motors disarmed due to inactivity"));
                    armed = Armed::Disarmed;
                    channel_pwms = STOP_PWMS;
                }

                // The escs like being sent 1500 us for a little bit before we start sending them
                // the actual speeds
                if matches!(armed, Armed::Armed) && last_rearm_timestamp.elapsed() < arming_duration
                {
                    channel_pwms = STOP_PWMS;
                }

                // Sync state with pwm chip
                match armed {
                    Armed::Armed => {
                        pwm_controller.output_enable();
                    }
                    Armed::Disarmed => {
                        pwm_controller.output_disable();
                        channel_pwms = STOP_PWMS;
                    }
                }

                trace!(?armed, ?channel_pwms, "Writing Pwms");

                // Write the current pwms to the pwm chip
                let rst = pwm_controller
                    .set_pwms(channel_pwms)
                    .context("Could not communicate with PCA9685");

                if let Err(err) = rst {
                    warn!("Could not write pwms");

                    let _ = errors.send(err);
                }

                if last_armed != armed {
                    info!("PWM Chip: {armed:?}");

                    last_armed = armed;
                }

                span.exit();

                deadline += interval;
                let remaining = deadline - Instant::now();
                thread::sleep(remaining);
            }
        })
        .context("Spawn thread")?;

    Ok(())
}

fn listen_to_pwms(
    channels: Res<GenericMotorIds>,
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
        .send(PwmEvent::Arm(*armed))
        .context("Send data to pwm thread")?;

    let mut channel_batch = STOP_SIGNALS;
    for (RobotId(robot_net_id), &channel, &signal, raw_range) in &pwms {
        if robot_net_id != net_id {
            continue;
        }

        let LocalMotorId::PwmChannel(channel) = channel.into() else {
            continue;
        };

        let pwm = match signal {
            MotorSignal::Percent(pct) => raw_range.raw_from_percent(pct),
            MotorSignal::Raw(raw) => raw,
        };
        let pwm = raw_range.clamp_raw(pwm) as u16;

        let id = channel.id() as usize;
        if id < NUM_CHANNELS {
            channel_batch[id] = pwm;
        } else {
            warn!("Attempted to drive unknown pwm channel {id}");
        }
    }

    channels
        .0
        .send(PwmEvent::Batch(channel_batch))
        .context("Send data to pwm thread")?;

    Ok(())
}

fn shutdown(channels: Res<GenericMotorIds>, mut exit: EventReader<AppExit>) {
    for _event in exit.read() {
        let _ = channels.0.send(PwmEvent::Shutdown);
    }
}
