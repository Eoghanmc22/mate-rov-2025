use std::{
    io,
    net::{IpAddr, SocketAddr},
    process::{Child, Command},
    thread,
    time::Duration,
};

use ahash::{HashMap, HashSet};
use anyhow::{anyhow, bail, Context};
use bevy::{core::Name, transform::components::Transform};
use common::{
    bundles::CameraBundle,
    components::{CameraDefinition, RobotId},
};
use crossbeam::channel::Sender;
use thiserror::Error;
use tracing::{info, warn};
use v4l::{
    capability, frameinterval::FrameIntervalEnum, framesize::FrameSizeEnum, video::Capture, Device,
    FourCC,
};

use crate::{
    config::CameraConfigDefinition, plugins::sensors::cameras::gstreamer::GstCameraDevice,
};

use super::gstreamer::GstCamera;

pub struct CameraHandler<'a> {
    // TODO: Remove?
    target_ip: Option<IpAddr>,
    cameras_processes: HashMap<String, CameraStreamProcess>,
    next_port: u16,
    tx_cameras: Sender<Vec<CameraBundle>>,
    robot: RobotId,
    camera_config: CameraConfigDefinition,
    error_consumer: &'a dyn Fn(anyhow::Error),
}

#[derive(Debug)]
struct CameraStreamProcess {
    process: Child,
    target: SocketAddr,
    camera: GstCamera,
}

#[derive(Debug, Clone)]
struct EnumeratedCamera {
    camera: GstCamera,
    name: String,
}

#[derive(Error, Debug)]
pub enum FatalError {
    #[error("Communication channel closed")]
    ChannelDisconnected,
    // #[error(transparent)]
    // Other(#[from] anyhow::Error),
}

#[derive(Debug, Clone, Copy)]
pub struct ChannelDisconnected;

// FIXME: I dont like how we handle errors here
impl<'a> CameraHandler<'a> {
    pub fn new(
        tx_cameras: Sender<Vec<CameraBundle>>,
        robot: RobotId,
        camera_config: CameraConfigDefinition,
        error_consumer: &'a dyn Fn(anyhow::Error),
    ) -> Self {
        Self {
            target_ip: None,
            cameras_processes: HashMap::default(),
            next_port: 1024,
            tx_cameras,
            robot,
            camera_config,
            error_consumer,
        }
    }

    pub fn update_peer(&mut self, addrs: Option<SocketAddr>) -> Result<(), FatalError> {
        self.target_ip = addrs.map(|it| it.ip());

        match addrs {
            Some(_) => self.restart_streams(),
            None => self.kill_streams(),
        }
    }

    pub fn restart_streams(&mut self) -> Result<(), FatalError> {
        info!("Camera thread new peer");

        self.kill_all_camera_streams();

        self.spawn_camera_streams();

        self.report_avaible_cameras()?;

        Ok(())
    }

    pub fn kill_streams(&mut self) -> Result<(), FatalError> {
        info!("Camera thread lost peer");

        self.target_ip = None;

        self.kill_all_camera_streams();

        self.report_avaible_cameras()?;

        Ok(())
    }

    //
    // BEGIN SHARED IMPL
    //

    fn spawn_camera_streams(&mut self) {
        let Some(addrs) = self.target_ip else {
            (self.error_consumer)(
                anyhow!("Attempted to spawn camera streams without a peer").into(),
            );
            return;
        };

        for camera in self.enumerate_cameras() {
            let rst = self
                .add_camera(camera.name.clone(), camera.camera, addrs)
                .with_context(|| format!("Start gstreamer for {}", camera.name));

            if let Err(err) = rst {
                (self.error_consumer)(err);
            }
        }
    }

    fn kill_all_camera_streams(&mut self) {
        for (camera, mut stream) in self.cameras_processes.drain() {
            let rst = stream
                .process
                .kill()
                .with_context(|| format!("Kill gstreamer for {camera}"));

            if let Err(err) = rst {
                (self.error_consumer)(err);
            }

            let rst = stream
                .process
                .wait()
                .with_context(|| format!("Wait gstreamer for {camera}"));

            if let Err(err) = rst {
                (self.error_consumer)(err);
            }
        }

        // TODO: Is this needed?
        thread::sleep(Duration::from_millis(500));
    }

    fn report_avaible_cameras(&self) -> Result<(), FatalError> {
        let camera_list = self.camera_list();

        self.tx_cameras
            .send(camera_list)
            .map_err(|_| FatalError::ChannelDisconnected)?;

        Ok(())
    }
    /// Starts a gstreamer and updates state
    fn add_camera(&mut self, name: String, camera: GstCamera, ip: IpAddr) -> anyhow::Result<()> {
        // TODO: Replace?
        // if let Some(path) = camera.device.device_path() {
        //     let setup_exit = Command::new("/home/pi/mate/setup_camera.sh")
        //         .arg(path)
        //         .spawn()
        //         .context("Setup cameras")?
        //         .wait()
        //         .context("wait on setup")?;
        //     if !setup_exit.success() {
        //         bail!("Could not setup cameras");
        //     }
        // }

        let bind = (ip, self.next_port).into();
        let child = camera
            .start_gstreamer(bind)
            .with_context(|| format!("Spawn gstreamer for {}", name))?;
        self.next_port += 1;

        self.cameras_processes.insert(
            name,
            CameraStreamProcess {
                process: child,
                target: bind,
                camera,
            },
        );

        Ok(())
    }
    /// Converts internal repersentation of cameras to what the protocol calls for
    fn camera_list(&self) -> Vec<CameraBundle> {
        let mut list = Vec::new();

        for (name, stream) in &self.cameras_processes {
            let (name, transform) = match self.camera_config.cameras.get(name) {
                Some(definition) => (
                    format!("{} ({:?})", name, definition.camera),
                    definition.transform.flatten(),
                ),
                None => (name.to_owned(), Transform::default()),
            };

            list.push(CameraBundle {
                name: Name::new(name),
                camera: CameraDefinition {
                    preliminary_pipeline: stream.camera.client_pipeline(stream.target),
                },
                robot: self.robot,
                transform,
            });
        }

        list
    }

    // TODO: rewrite using the v4l2 rust crate
    fn enumerate_cameras(&self) -> Vec<EnumeratedCamera> {
        let mut enumerated_cameras = vec![];

        let devices = v4l::context::enum_devices();
        for device in devices {
            let device_path = device.path();
            let Some(device_path_str) = device_path.to_str() else {
                (self.error_consumer)(anyhow!(
                    "Could not convert v4l device path to str: {}",
                    device_path.to_string_lossy()
                ));
                continue;
            };

            info!("Enumerating camera at {device_path_str}");

            let device = Device::with_path(device_path).context("V4l device from path");
            let device = match device {
                Ok(device) => device,
                Err(err) => {
                    (self.error_consumer)(err);
                    continue;
                }
            };

            let caps = device.query_caps().context("Query v4l device capabilities");
            let caps = match dbg!(caps) {
                Ok(caps) => caps,
                Err(err) => {
                    (self.error_consumer)(err);
                    continue;
                }
            };

            if !caps.capabilities.contains(capability::Flags::VIDEO_CAPTURE) {
                continue;
            }

            dbg!(device.query_controls());
            let formats = device.enum_formats().context("Enumerate formats");
            let formats = match dbg!(formats) {
                Ok(formats) => formats,
                Err(err) => {
                    (self.error_consumer)(err);
                    continue;
                }
            };

            let mut supports_h264 = false;
            let mut supports_mjpeg = false;

            for format in formats {
                if format.fourcc == FourCC::new(b"H264") {
                    supports_h264 = true;
                }
                if format.fourcc == FourCC::new(b"MJPG") {
                    supports_mjpeg = true;
                }
            }

            let mut best_format = if supports_h264 {
                GstCameraDevice::H264V4l2 {
                    device: device_path_str.to_owned(),
                }
            } else if supports_mjpeg {
                GstCameraDevice::MjpegV4l2 {
                    device: device_path_str.to_owned(),
                }
            } else {
                // TODO: Should this use the error consumer?
                warn!("Camera at {} has no supported formats", device_path_str);
                continue;
            };

            let desired_config =
                self.camera_config.cameras.iter().find(|(_, config)| {
                    config.camera.device.device_path() == Some(device_path_str)
                });
            if let Some((_, desired_config)) = desired_config {
                let supported = match desired_config.camera.device {
                    GstCameraDevice::H264V4l2 { .. } => supports_h264,
                    GstCameraDevice::MjpegV4l2 { .. } => supports_mjpeg,
                    GstCameraDevice::Test => false,
                };

                if supported {
                    best_format = desired_config.camera.device.clone();
                } else {
                    // TODO: Should this use the error consumer?
                    warn!(
                        "Camera at {} does not support the desited format",
                        device_path_str
                    );
                }
            }

            let format_code = match best_format {
                GstCameraDevice::H264V4l2 { .. } => FourCC::new(b"H264"),
                GstCameraDevice::MjpegV4l2 { .. } => FourCC::new(b"MJPG"),
                GstCameraDevice::Test => unreachable!(),
            };

            let frame_sizes = device
                .enum_framesizes(format_code)
                .context("Enum frame sizes");
            let frame_sizes = match dbg!(frame_sizes) {
                Ok(frame_sizes) => frame_sizes,
                Err(err) => {
                    (self.error_consumer)(err);
                    continue;
                }
            };

            let Some(mut best_frame_size) = frame_sizes
                .iter()
                .map(|it| match &it.size {
                    FrameSizeEnum::Discrete(discrete) => (discrete.width, discrete.height),
                    FrameSizeEnum::Stepwise(stepwise) => (stepwise.max_width, stepwise.max_height),
                })
                .max_by_key(|it| it.0 * it.1)
            else {
                (self.error_consumer)(anyhow!(
                    "Camera at {device_path_str} has no frame sizes for {format_code}",
                ));
                continue;
            };

            if let Some((_, desired_config)) = desired_config {
                let mut found_desired_size = false;

                for frame_size in frame_sizes {
                    match frame_size.size {
                        FrameSizeEnum::Discrete(ref discrete) => {
                            if discrete.width == desired_config.camera.width
                                && discrete.height == desired_config.camera.height
                            {
                                found_desired_size = true;
                                break;
                            }
                        }
                        FrameSizeEnum::Stepwise(ref stepwise) => {
                            if !(stepwise.min_width..=stepwise.max_width)
                                .contains(&desired_config.camera.width)
                                || !(stepwise.min_width..=stepwise.max_width)
                                    .contains(&desired_config.camera.width)
                            {
                                continue;
                            }

                            let valid_width = (desired_config.camera.width - stepwise.min_width)
                                % stepwise.step_width
                                == 0;
                            let valid_height = (desired_config.camera.height - stepwise.min_height)
                                % stepwise.step_height
                                == 0;

                            if valid_width && valid_height {
                                found_desired_size = true;
                                break;
                            }
                        }
                    }
                }

                if found_desired_size {
                    best_frame_size = (desired_config.camera.width, desired_config.camera.height);
                } else {
                    // TODO: Should this use the error consumer?
                    warn!(
                        "Camera at {} does not support the desited frame size",
                        device_path_str
                    );
                }
            }

            let (width, height) = best_frame_size;

            let frame_rates = device
                .enum_frameintervals(format_code, width, height)
                .context("Enum frame rates");
            let frame_rates = match dbg!(frame_rates) {
                Ok(frame_rates) => frame_rates,
                Err(err) => {
                    (self.error_consumer)(err);
                    continue;
                }
            };

            let Some(mut best_frame_rate) = frame_rates
                .iter()
                .map(|it| match &it.interval {
                    // Interval is repricial of frame rate
                    FrameIntervalEnum::Discrete(discrete) => {
                        (discrete.denominator, discrete.numerator)
                    }
                    FrameIntervalEnum::Stepwise(stepwise) => {
                        (stepwise.max.denominator, stepwise.max.numerator)
                    }
                })
                .max_by(|a, b| u32::cmp(&(a.0 * b.1), &(b.0 * a.1)))
            else {
                (self.error_consumer)(anyhow!(
                    "Camera at {device_path_str} has no frame rates for {format_code} at {width}x{height}",
                ));
                continue;
            };

            if let Some((_, desired_config)) = desired_config {
                let mut found_desired_rate = false;

                for frame_rate in frame_rates {
                    match frame_rate.interval {
                        FrameIntervalEnum::Discrete(ref discrete) => {
                            if discrete.numerator == desired_config.camera.frame_rate.0
                                && discrete.denominator == desired_config.camera.frame_rate.1
                            {
                                found_desired_rate = true;
                                break;
                            }
                        }
                        FrameIntervalEnum::Stepwise(ref _stepwise) => {
                            unimplemented!();
                        }
                    }
                }

                if found_desired_rate {
                    best_frame_rate = (
                        desired_config.camera.frame_rate.0,
                        desired_config.camera.frame_rate.1,
                    );
                } else {
                    // TODO: Should this use the error consumer?
                    warn!(
                        "Camera at {} does not support the desited frame rate",
                        device_path_str
                    );
                }
            }

            let frame_rate = best_frame_rate;
            // TODO: "best_format" is bad naming
            let device = best_format;

            enumerated_cameras.push(EnumeratedCamera {
                camera: GstCamera {
                    width,
                    height,
                    frame_rate,
                    device,
                },
                name: if let Some((name, _)) = desired_config {
                    name.to_owned()
                } else {
                    device_path_str.to_owned()
                },
            });
        }

        for (name, config) in &self.camera_config.cameras {
            if let None = config.camera.device.device_path() {
                enumerated_cameras.push(EnumeratedCamera {
                    camera: config.camera.to_owned(),
                    name: name.to_owned(),
                });
            }
        }

        enumerated_cameras
    }
}

impl Drop for CameraHandler<'_> {
    fn drop(&mut self) {
        let _ = self.kill_streams();
    }
}
