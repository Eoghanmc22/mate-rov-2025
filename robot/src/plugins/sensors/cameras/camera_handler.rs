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
use tracing::info;

use crate::config::CameraConfigDefinition;

pub struct CameraHandler<'a> {
    // TODO: Remove?
    target_ip: Option<IpAddr>,
    // TODO: Remove?
    enumerated_cameras: HashSet<String>,
    cameras_processes: HashMap<String, (Child, SocketAddr)>,
    next_port: u16,
    tx_cameras: Sender<Vec<CameraBundle>>,
    robot: RobotId,
    camera_config: CameraConfigDefinition,
    error_consumer: &'a dyn Fn(anyhow::Error),
}

#[derive(Error, Debug)]
pub enum FatalError {
    #[error("Communication channel closed")]
    ChannelDisconnected,
    // #[error(transparent)]
    // Other(#[from] anyhow::Error),
}

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
            enumerated_cameras: HashSet::default(),
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

        let cameras = enumerate_cameras().context("Enumerate Cameras");
        let cameras = match cameras {
            Ok(cameras) => cameras,
            Err(err) => {
                (self.error_consumer)(err);
                return Ok(());
            }
        };
        self.enumerated_cameras = cameras;

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

        for camera in &self.enumerated_cameras {
            let rst = add_camera(
                camera,
                addrs,
                &mut self.cameras_processes,
                &mut self.next_port,
            )
            .with_context(|| format!("Start gstreamer for {camera}"));

            if let Err(err) = rst {
                (self.error_consumer)(err);
            }
        }
    }

    fn kill_all_camera_streams(&mut self) {
        for (camera, (mut child, _)) in self.cameras_processes.drain() {
            let rst = child
                .kill()
                .with_context(|| format!("Kill gstreamer for {camera}"));

            if let Err(err) = rst {
                (self.error_consumer)(err);
            }

            let rst = child
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
        let camera_list = camera_list(&self.cameras_processes, self.robot, &self.camera_config);

        self.tx_cameras
            .send(camera_list)
            .map_err(|_| FatalError::ChannelDisconnected)?;

        Ok(())
    }
}

impl Drop for CameraHandler<'_> {
    fn drop(&mut self) {
        let _ = self.kill_streams();
    }
}

// TODO: rewrite using the v4l2 rust crate
fn enumerate_cameras() -> anyhow::Result<HashSet<String>> {
    let output = Command::new("/home/pi/mate/detect_cameras.sh")
        .output()
        .context("Run detect_cameras.sh")?;

    if !output.status.success() {
        bail!("Collect cameras: {}", output.status)
    }

    let data = str::from_utf8(&output.stdout).context("Parse output of detect_cameras.sh")?;
    let cameras = data.lines().map(ToOwned::to_owned).collect();

    Ok(cameras)
}

/// Spawns a gstreamer with the args necessary
fn start_gstreamer(camera: &str, addrs: SocketAddr) -> io::Result<Child> {
    Command::new("gst-launch-1.0")
        .arg("v4l2src")
        .arg(format!("device={camera}"))
        .arg("do-timestamp=true")
        .arg("!")
        .arg("h264parse")
        .arg("!")
        .arg("video/x-h264,stream-format=avc,alignment=au,width=1920,height=1080,framerate=30/1")
        .arg("!")
        .arg("rtph264pay")
        .arg("aggregate-mode=zero-latency")
        .arg("config-interval=10")
        .arg("pt=96")
        .arg("!")
        .arg("udpsink")
        .arg("sync=false")
        .arg(format!("host={}", addrs.ip()))
        .arg(format!("port={}", addrs.port()))
        .spawn()
}

/// Starts a gstreamer and updates state
fn add_camera(
    camera: &str,
    ip: IpAddr,
    cameras: &mut HashMap<String, (Child, SocketAddr)>,
    port: &mut u16,
) -> anyhow::Result<()> {
    let setup_exit = Command::new("/home/pi/mate/setup_camera.sh")
        .arg(camera)
        .spawn()
        .context("Setup cameras")?
        .wait()
        .context("wait on setup")?;
    if !setup_exit.success() {
        bail!("Could not setup cameras");
    }

    let bind = (ip, *port).into();
    let child =
        start_gstreamer(camera, bind).with_context(|| format!("Spawn gstreamer for {camera}"))?;
    *port += 1;

    cameras.insert((*camera).to_owned(), (child, bind));

    Ok(())
}

/// Converts internal repersentation of cameras to what the protocol calls for
fn camera_list(
    cameras: &HashMap<String, (Child, SocketAddr)>,
    robot: RobotId,
    camera_config: &CameraConfigDefinition,
) -> Vec<CameraBundle> {
    let mut list = Vec::new();

    for (name, &(_, location)) in cameras {
        let (name, transform) = match camera_config.cameras.get(name) {
            Some(definition) => (
                format!("{} ({})", definition.name, name),
                definition.transform.flatten(),
            ),
            None => (name.to_owned(), Transform::default()),
        };

        list.push(CameraBundle {
            name: Name::new(name),
            camera: CameraDefinition { location },
            robot,
            transform,
        });
    }

    list
}
