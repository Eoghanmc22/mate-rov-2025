use std::{
    io,
    net::SocketAddr,
    process::{Child, Command},
};

use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub struct GstCamera {
    pub width: u32,
    pub height: u32,
    // TODO: make a fraction type
    // (num, denom)
    pub frame_rate: (u32, u32),
    pub device: GstCameraDevice,
}

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub enum GstCameraDevice {
    H264V4l2 { device: String },
    MjpegV4l2 { device: String },
    Test,
}

// TODO: Can this be made better by using the gstreamer bindings?
impl GstCamera {
    pub fn start_gstreamer(&self, target: SocketAddr) -> io::Result<Child> {
        let Self {
            width,
            height,
            frame_rate,
            device,
        } = self;

        match device {
            GstCameraDevice::H264V4l2 { device } => {
                Command::new("gst-launch-1.0")
                    .arg("v4l2src")
                    .arg(format!("device={device}"))
                    // TODO: Determine if this is needed
                    .arg("do-timestamp=true")
                    .arg("!")
                    .arg("h264parse")
                    .arg("!")
                    .arg(format!("video/x-h264,stream-format=avc,alignment=au,width={width},height={height},framerate={}/{}", frame_rate.0, frame_rate.1))
                    .arg("!")
                    .arg("rtph264pay")
                    .arg("aggregate-mode=zero-latency")
                    .arg("config-interval=10")
                    .arg("pt=96")
                    .arg("!")
                    .arg("udpsink")
                    .arg("sync=false")
                    .arg(format!("host={}", target.ip()))
                    .arg(format!("port={}", target.port()))
                    .spawn()
            }
            GstCameraDevice::MjpegV4l2 { device } => {
                Command::new("gst-launch-1.0")
                    .arg("v4l2src")
                    .arg(format!("device={device}"))
                    // TODO: Determine if this is needed
                    .arg("do-timestamp=true")
                    .arg("!")
                    .arg(format!(
                        "image/jpeg,width={width},height={height},framerate={}/{}",
                        frame_rate.0, frame_rate.1
                    ))
                    .arg("!")
                    .arg("jpegparse")
                    .arg("!")
                    .arg("rtpjpegpay")
                    .arg("pt=26")
                    .arg("!")
                    .arg("udpsink")
                    .arg("sync=false")
                    .arg("buffer-size=30720")
                    .arg(format!("host={}", target.ip()))
                    .arg(format!("port={}", target.port()))
                    .spawn()
            }
            GstCameraDevice::Test => {
                dbg!(Command::new("gst-launch-1.0")
                    .arg("videotestsrc")
                    // TODO: Determine if this is needed
                    .arg("do-timestamp=true")
                    .arg("!")
                    .arg(format!(
                        "video/x-raw,format=I420,width={width},height={height},framerate={}/{}",
                        frame_rate.0, frame_rate.1
                    ))
                    .arg("!")
                    .arg("jpegenc")
                    // .arg("avenc_mjpeg")
                    .arg("!")
                    .arg("rtpjpegpay")
                    .arg("pt=26")
                    .arg("!")
                    // .arg("fakesink"))
                    .arg("udpsink")
                    .arg("sync=false")
                    .arg("buffer-size=104857600")
                    .arg(format!("host={}", target.ip()))
                    .arg(format!("port={}", target.port())))
                .spawn()
            }
        }
    }

    pub fn client_pipeline(&self, src: SocketAddr) -> String {
        let Self {
            width,
            height,
            frame_rate,
            device,
        } = self;

        let ip = src.ip();
        let port = src.port();

        match device {
            GstCameraDevice::H264V4l2 { .. } => {
                format!("udpsrc address={ip} port={port} caps=application/x-rtp,payload=96 ! ")
                    + "rtph264depay ! "
                    + "avdec_h264 discard-corrupted-frames=true "
            }
            GstCameraDevice::MjpegV4l2 { .. } | GstCameraDevice::Test => {
                format!("udpsrc address={ip} port={port} caps=application/x-rtp,payload=26 buffer-size=104857600 ! ")
                    + "rtpjpegdepay ! "
                    + "avdec_mjpeg discard-corrupted-frames=true "
            }
        }
    }
}

impl GstCameraDevice {
    pub fn device_path(&self) -> Option<&str> {
        match self {
            GstCameraDevice::H264V4l2 { device } | GstCameraDevice::MjpegV4l2 { device } => {
                Some(device)
            }
            GstCameraDevice::Test => None,
        }
    }
}
