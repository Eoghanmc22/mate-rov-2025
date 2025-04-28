use core::f32;

use anyhow::{bail, Context};
use bevy::{
    app::{App, Plugin},
    ecs::world::{EntityRef, EntityWorldMut, World},
    math::{Quat, Vec3},
};
use common::components::{Orientation, OrientationTarget, Robot, RobotId};
use opencv::{
    core::{MatExpr, MatTraitConst, MatTraitConstManual, ToInputArray, Vector},
    imgcodecs, imgproc,
    prelude::{Mat, StitcherTrait},
    stitching::{Stitcher, Stitcher_Mode, Stitcher_Status},
    sys::cv_detail_stitchingLogLevel,
};
use tracing::{info, warn};

use super::{AppPipelineExt, Pipeline, PipelineCallbacks};

const ORIENTATION_TOLERANCE: f32 = 10.0f32.to_radians();
const SHARPNESS_THRSHOLD: f32 = 100.0;

pub struct PhotoSpherePipelinePlugin;

impl Plugin for PhotoSpherePipelinePlugin {
    fn build(&self, app: &mut App) {
        app.register_video_pipeline::<PhotoSpherePipeline>("Photo Sphere Pipeline");
    }
}

pub struct PhotoSpherePipeline {
    state: PhotoSpherePipelineState,
    remaining_targets: Vec<Quat>,
    bw: Mat,
    laplacian: Mat,
    images: Vector<Mat>,
    starting_orientation_target: Option<Quat>,
}

impl Default for PhotoSpherePipeline {
    fn default() -> Self {
        Self {
            state: PhotoSpherePipelineState::default(),
            remaining_targets: fibonacci_sphere(50),
            bw: Mat::default(),
            laplacian: Mat::default(),
            images: Vector::default(),
            starting_orientation_target: None,
        }
    }
}

#[derive(Default, Clone, Copy)]
enum PhotoSpherePipelineState {
    #[default]
    Init,
    SelectNextTarget,
    WaitReachTarget(Quat),
    TakePhoto,
    Stitch,
}

impl Pipeline for PhotoSpherePipeline {
    // (_, _, is_valid)
    type Input = (Option<Orientation>, Option<OrientationTarget>, bool);

    fn collect_inputs(world: &World, entity: &EntityRef) -> Self::Input {
        let res: Option<Self::Input> = try {
            // Get id of attached robot
            let robot_id = entity.get::<RobotId>()?;

            // Find which entity is a robot and has that id
            let robot = world.iter_entities().find(|entity| {
                entity.contains::<Robot>() && entity.get::<RobotId>() == Some(robot_id)
            })?;

            (
                robot.get::<Orientation>().copied(),
                robot.get::<OrientationTarget>().copied(),
                true,
            )
        };

        res.unwrap_or_default()
    }

    fn process<'b, 'a: 'b>(
        &'a mut self,
        cmds: &mut PipelineCallbacks,
        data: &Self::Input,
        img: &'b mut Mat,
    ) -> anyhow::Result<&'b mut Mat> {
        match self.state {
            PhotoSpherePipelineState::Init => {
                if let (_, origional_target, true) = data {
                    self.starting_orientation_target = origional_target.map(|it| it.0);
                    self.state = PhotoSpherePipelineState::SelectNextTarget;
                }
            }
            PhotoSpherePipelineState::SelectNextTarget => {
                let Some(orientation) = data.0 else {
                    cmds.should_end();
                    bail!("Robot does not have orientation");
                };

                let target = self
                    .remaining_targets
                    .iter()
                    .enumerate()
                    .map(|(idx, target)| (idx, *target, target.angle_between(orientation.0).abs()))
                    .min_by(|a, b| f32::total_cmp(&a.2, &b.2));

                // let target = self.remaining_targets.pop();
                if let Some((idx, target, _distance)) = target {
                    self.remaining_targets.remove(idx);

                    self.state = PhotoSpherePipelineState::WaitReachTarget(target);
                    // TODO: When we get faliable systems in bevy 0.16, use the ? operator here
                    cmds.pipeline(move |entity| {
                        // Get id of attached robot
                        let Some(robot_id) = entity.get::<RobotId>().copied() else {
                            warn!("PhotoSpherePipeline does not have a RobotId");
                            return;
                        };

                        let world = entity.into_world_mut();

                        // Find which entity is a robot and has that id
                        let robot = world
                            .iter_entities()
                            .find(|entity| {
                                entity.contains::<Robot>()
                                    && entity.get::<RobotId>().copied() == Some(robot_id)
                            })
                            .map(|it| it.id());

                        let Some(mut robot) = robot.and_then(|it| world.get_entity_mut(it).ok())
                        else {
                            warn!("PhotoSpherePipeline's robot does not exist in world");
                            return;
                        };

                        robot.insert(OrientationTarget(target));
                    });
                } else {
                    self.state = PhotoSpherePipelineState::Stitch;
                }
            }
            PhotoSpherePipelineState::WaitReachTarget(quat) => {
                if let (Some(observed_orientation), _, true) = data {
                    if quat.angle_between(observed_orientation.0).abs() < ORIENTATION_TOLERANCE {
                        self.state = PhotoSpherePipelineState::TakePhoto;
                    } else {
                        info!(
                            "Angle error: {:.2}",
                            quat.angle_between(observed_orientation.0)
                                .abs()
                                .to_degrees()
                        );
                    }
                } else {
                    warn!("PhotoSpherePipeline has no orientation observation");
                }
            }
            PhotoSpherePipelineState::TakePhoto => {
                imgproc::cvt_color_def(img, &mut self.bw, imgproc::COLOR_BGR2GRAY)
                    .context("Convert color")?;
                imgproc::laplacian_def(&self.bw, &mut self.laplacian, opencv::core::CV_32F)
                    .context("Laplacian")?;

                // TODO: It would be more robust to use the 99th percentile sharpness rather than
                // the max sharpness
                let sharpness = self
                    .laplacian
                    .iter::<f32>()
                    .context("Image Iter")?
                    .map(|(_point, val)| val.abs())
                    .max_by(f32::total_cmp)
                    .unwrap_or_default();

                info!("Image sharpness: {sharpness:?}");

                if sharpness > SHARPNESS_THRSHOLD {
                    imgcodecs::imwrite_def(&format!("_pano/img{:4}.jpg", self.images.len()), &*img)
                        .context("Save stiched pano")?;
                    self.images.push(img.try_clone().context("Try clone")?);
                    self.state = PhotoSpherePipelineState::SelectNextTarget;
                }
            }
            PhotoSpherePipelineState::Stitch => {
                cmds.should_end();

                let mut pano = Mat::default();

                let mut sticher =
                    Stitcher::create(Stitcher_Mode::PANORAMA).context("Create sticher")?;
                let res = sticher
                    .stitch(&self.images, &mut pano)
                    .context("Stitch pano")?;

                match res {
                    Stitcher_Status::OK => {}
                    Stitcher_Status::ERR_NEED_MORE_IMGS => {
                        bail!("Stiching failed due to lack of images")
                    }
                    Stitcher_Status::ERR_HOMOGRAPHY_EST_FAIL => {
                        bail!("Stiching failed due error during homography Estimation")
                    }
                    Stitcher_Status::ERR_CAMERA_PARAMS_ADJUST_FAIL => {
                        bail!("Stiching failed due to inconsistant camera params")
                    }
                }

                imgcodecs::imwrite_def("_pano/pano.jpg", &pano).context("Save stiched pano")?;
            }
        }

        Ok(img)
    }

    fn cleanup(self, entity: &mut EntityWorldMut) {
        // Get id of attached robot
        let Some(robot_id) = entity.get::<RobotId>().copied() else {
            warn!("PhotoSpherePipeline does not have a RobotId");
            return;
        };

        entity.world_scope(|world| {
            // Find which entity is a robot and has that id
            let robot = world
                .iter_entities()
                .find(|entity| {
                    entity.contains::<Robot>() && entity.get::<RobotId>().copied() == Some(robot_id)
                })
                .map(|it| it.id());

            let Some(mut robot) = robot.and_then(|it| world.get_entity_mut(it).ok()) else {
                warn!("PhotoSpherePipeline's robot does not exist in world");
                return;
            };

            if let Some(target) = self.starting_orientation_target {
                robot.insert(OrientationTarget(target));
            } else {
                robot.remove::<OrientationTarget>();
            }
        });
    }
}

pub fn fibonacci_sphere(samples: usize) -> Vec<Quat> {
    let mut points = vec![];

    let phi = f32::consts::PI * (5f32.sqrt() - 1.0); // golden angle in radians

    for i in 0..samples {
        let z = 0.7 - (i as f32 / (samples as f32 - 1.0)) * 1.4; // y goes from 1 to -1
        let radius = f32::sqrt(1.0 - z * z); // radius at y

        let theta = phi * i as f32; // golden angle increment

        let x = theta.cos() * radius;
        let y = theta.sin() * radius;

        points.push(Quat::from_rotation_arc(Vec3::Y, Vec3::new(x, y, z)));
    }

    points
}
