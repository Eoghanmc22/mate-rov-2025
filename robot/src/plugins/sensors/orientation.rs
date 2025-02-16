mod kalman_filter;

use std::{
    iter, thread,
    time::{Duration, Instant},
};

use ahrs::{Ahrs, Madgwick};
use anyhow::{anyhow, Context};
use bevy::{app::AppExit, prelude::*};
use common::{
    components::{FilteredPose, Inertial, Magnetic, Orientation, Pose, RawPose},
    error::{self, ErrorEvent, Errors},
    events::ResetYaw,
    types::hw::{InertialFrame, MagneticFrame},
};
use crossbeam::channel::{self, Receiver, Sender};
use glam::Vec3A;
use kalman_filter::{
    accel, accel_position, create_initial_state, observation_from_measurement, position,
    state_constants, KalmanConfig, KalmanFilter, KalmanState, Measurement, ROVTransitionModel,
};
use nalgebra::{OVector, Vector3};
use tracing::{span, Level};

use crate::{
    peripheral::{icm20602::Icm20602, mmc5983::Mcc5983},
    plugins::core::robot::LocalRobot,
};

const USE_MAGNETOMETER: bool = false;

pub struct OrientationPlugin;

impl Plugin for OrientationPlugin {
    fn build(&self, app: &mut App) {
        let orientation_offset = Quat::from_euler(
            EulerRot::YXZ,
            90.0f32.to_radians(),
            0.0f32.to_radians(),
            0.0f32.to_radians(),
        );
        let mut madgwick = Madgwick::new(1.0 / 1000.0, 0.041);
        // let mut madgwick = Madgwick::new(1.0 / 1000.0, 0.41);
        madgwick.quat = orientation_offset.into();

        app.insert_resource(OrientationOffset(orientation_offset));
        app.insert_resource(MadgwickFilter(madgwick));
        app.insert_resource(KalmanStateRes(create_initial_state()));

        app.add_systems(Startup, start_inertial_thread.pipe(error::handle_errors));
        app.add_systems(
            PreUpdate,
            (
                reset_yaw_handler.before(read_new_data),
                read_new_data.run_if(resource_exists::<InertialChannels>),
            ),
        );
        app.add_systems(Last, shutdown.run_if(resource_exists::<InertialChannels>));
    }
}

#[derive(Resource)]
struct InertialChannels(
    Receiver<([InertialFrame; 10], [MagneticFrame; 1])>,
    Sender<()>,
);

#[derive(Resource)]
struct MadgwickFilter(Madgwick<f32>);

#[derive(Resource)]
struct KalmanStateRes(KalmanState);

#[derive(Resource)]
struct OrientationOffset(Quat);

fn start_inertial_thread(mut cmds: Commands, errors: Res<Errors>) -> anyhow::Result<()> {
    let (tx_data, rx_data) = channel::bounded(5);
    let (tx_exit, rx_exit) = channel::bounded(1);

    let mut imu = Icm20602::new(Icm20602::SPI_BUS, Icm20602::SPI_SELECT, Icm20602::SPI_CLOCK)
        .context("Inerital Sensor (ICM20602)")?;
    let mut mag = Mcc5983::new(Mcc5983::SPI_BUS, Mcc5983::SPI_SELECT, Mcc5983::SPI_CLOCK)
        .context("Magnmetic Sensor (MCC5983)")?;

    cmds.insert_resource(InertialChannels(rx_data, tx_exit));

    let errors = errors.0.clone();
    thread::Builder::new()
        .name("IMU Thread".to_owned())
        .spawn(move || {
            let _span = span!(Level::INFO, "IMU sensor thread").entered();

            let interval = Duration::from_secs_f32(1.0 / 1000.0);
            let counts = 10;

            let mut counter = 0;

            let mut inertial_buffer = [InertialFrame::default(); 10];
            let mut mag_buffer = [MagneticFrame::default(); 1];

            let inertial_divisor = counts / inertial_buffer.len();
            let mag_divisor = counts / mag_buffer.len();

            let mut deadline = Instant::now();

            let mut first_run = true;

            loop {
                let span = span!(Level::INFO, "IMU sensor cycle").entered();

                if counter == 0 && !first_run {
                    let res = tx_data.send((inertial_buffer, mag_buffer));
                    if res.is_err() {
                        // Peer disconnected
                        return;
                    }
                }

                if counter % inertial_divisor == 0 {
                    let rst = imu.read_frame().context("Read inertial frame");

                    match rst {
                        Ok(frame) => {
                            inertial_buffer[counter / inertial_divisor] = frame;
                        }
                        Err(err) => {
                            let _ = errors.send(err);
                        }
                    }
                }

                if counter % mag_divisor == 0 {
                    let rst = mag.read_frame().context("Read magnetic frame");

                    match rst {
                        Ok(frame) => {
                            mag_buffer[counter / mag_divisor] = frame;
                        }
                        Err(err) => {
                            let _ = errors.send(err);
                        }
                    }
                }

                if let Ok(()) = rx_exit.try_recv() {
                    return;
                }

                span.exit();

                deadline += interval;
                let remaining = deadline - Instant::now();
                thread::sleep(remaining);

                counter += 1;
                counter %= counts;
                first_run = false;
            }
        })
        .context("Spawn thread")?;

    Ok(())
}

fn read_new_data(
    mut cmds: Commands,
    channels: Res<InertialChannels>,
    mut madgwick_filter: ResMut<MadgwickFilter>,
    mut kalman_state: ResMut<KalmanStateRes>,
    orientation_offset: Res<OrientationOffset>,
    robot: Res<LocalRobot>,
    query: Query<Ref<RawPose>>,
    mut errors: EventWriter<ErrorEvent>,
) {
    let mut total_accel = Vec3A::ZERO;
    let mut sample_count = 0;

    for (inertial, magnetic) in channels.0.try_iter() {
        for (inertial, mut magnetic) in inertial.into_iter().zip(
            magnetic
                .into_iter()
                .map(Option::Some)
                .chain(iter::repeat(None)),
        ) {
            let gyro = Vector3::new(inertial.gyro_x.0, inertial.gyro_y.0, inertial.gyro_z.0)
                * (std::f32::consts::PI / 180.0);
            let accel = Vector3::new(inertial.accel_x.0, inertial.accel_y.0, inertial.accel_z.0);

            if !USE_MAGNETOMETER {
                magnetic = None;
            }

            let rst = if let Some(magnetic) = magnetic {
                let mag = Vector3::new(magnetic.mag_x.0, magnetic.mag_y.0, magnetic.mag_z.0);

                madgwick_filter.0.update(&gyro, &accel, &mag)
            } else {
                madgwick_filter.0.update_imu(&gyro, &accel)
            };

            if let Err(msg) = rst {
                errors.send(anyhow!("Process IMU frame: {msg:?}").into());
            }

            // Step kalman filter
            let quat: glam::Quat = madgwick_filter.0.quat.into();
            let accel =
                quat * Vec3A::new(
                    inertial.accel_x.0 * 9.81,
                    inertial.accel_y.0 * 9.81,
                    inertial.accel_z.0 * 9.81,
                ) - Vec3A::new(0.0, 0.0, 9.81);

            total_accel += accel;
            sample_count += 1;
        }

        if sample_count != 0 {
            let accel = total_accel / sample_count as f32;

            let config = KalmanConfig::default();
            let transition_model = ROVTransitionModel::new(&config, 1.0 / 100.0);
            let kalman_state = &mut kalman_state.0;

            match query.get(robot.entity) {
                Ok(raw_pose) if raw_pose.is_changed() => {
                    let observation_model = accel_position::ROVObservationModel::new(&config);
                    let kalman_filter = KalmanFilter::new(&transition_model, &observation_model);

                    if let Ok(new_state) = kalman_filter.step(
                        kalman_state,
                        &observation_from_measurement(Measurement {
                            accel,
                            pos: raw_pose.0.position,
                        }),
                    ) {
                        kalman_filter::print_state(&new_state);
                        *kalman_state = new_state;
                    }
                }
                _ => {
                    let observation_model = accel::ROVObservationModel::new(&config);
                    let kalman_filter = KalmanFilter::new(&transition_model, &observation_model);

                    if let Ok(new_state) = kalman_filter.step(
                        kalman_state,
                        &observation_from_measurement(Measurement {
                            accel,
                            pos: Vec3A::ZERO,
                        }),
                    ) {
                        *kalman_state = new_state;
                    }
                }
            }
        }

        let quat: glam::Quat = madgwick_filter.0.quat.into();
        let orientation = Orientation(quat * orientation_offset.0.inverse());

        let inertial = inertial.last().unwrap();
        let inertial = Inertial(*inertial);

        let magnetic = magnetic.last().unwrap();
        let magnetic = Magnetic(*magnetic);

        let state = kalman_state.0.state();
        let pos = Vec3A::new(
            state[state_constants::POS_X],
            state[state_constants::POS_Y],
            state[state_constants::POS_Z],
        );
        let velo = Vec3A::new(
            state[state_constants::VEL_X],
            state[state_constants::VEL_Y],
            state[state_constants::VEL_Z],
        );
        let acc = Vec3A::new(
            state[state_constants::ACC_X],
            state[state_constants::ACC_Y],
            state[state_constants::ACC_Z],
        );
        let acc_bias = Vec3A::new(
            state[state_constants::BIAS_ACC_X],
            state[state_constants::BIAS_ACC_Y],
            state[state_constants::BIAS_ACC_Z],
        );
        let filtered = FilteredPose {
            pose: Pose {
                position: pos,
                rotation: quat,
            },
            velo,
            acc,
            acc_bias,
        };

        cmds.entity(robot.entity)
            .insert((orientation, inertial, magnetic, filtered));
    }
}

fn reset_yaw_handler(
    mut events: EventReader<ResetYaw>,
    mut madgwick_filter: ResMut<MadgwickFilter>,
) {
    for _ in events.read() {
        info!("Resetting Yaw");

        madgwick_filter.0.quat.as_mut_unchecked().vector_mut()[2] = 0.0;
        madgwick_filter.0.quat.renormalize();
    }
}

fn shutdown(channels: Res<InertialChannels>, mut exit: EventReader<AppExit>) {
    for _event in exit.read() {
        let _ = channels.1.send(());
    }
}
