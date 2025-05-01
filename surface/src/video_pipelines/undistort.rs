use anyhow::Context;
use bevy::{
    app::{App, Plugin},
    ecs::component::Component,
    math::Mat3A,
    prelude::{Entity, EntityRef, EntityWorldMut, World},
};
use common::components::CameraCalibration;
use opencv::{
    calib3d,
    core::{Range, Rect, Size},
    imgproc,
    prelude::*,
};

use crate::video_pipelines::{AppPipelineExt, FromWorldEntity, Pipeline, PipelineCallbacks};

pub struct UndistortPipelinePlugin;

impl Plugin for UndistortPipelinePlugin {
    fn build(&self, app: &mut App) {
        app.register_video_pipeline::<UndistortPipeline>("Undistort Pipeline");
    }
}

#[derive(Component, Clone, Copy, Debug, Default)]
pub struct CroppedCameraMatrix {
    pub mat: Mat3A,
}

pub struct UndistortPipeline {
    undistorted: Mat,
    cropped: Mat,

    mtx: Mat,
    dist: Mat,

    remap: Option<RemapData>,

    // TODO: Is this approch the cleanest?
    camera_entity: Entity,
}

struct RemapData {
    size: Size,

    map_x: Mat,
    map_y: Mat,

    roi: Rect,
}

impl Pipeline for UndistortPipeline {
    type Input = ();

    fn collect_inputs(_world: &World, _entity: &EntityRef) -> Self::Input {
        // No-op
    }

    fn process<'b, 'a: 'b>(
        &'a mut self,
        cmds: &mut PipelineCallbacks,
        _data: &Self::Input,
        img: &'b mut Mat,
    ) -> anyhow::Result<&'b mut Mat> {
        let size = img.size().context("Get image size")?;

        if let Some(ref mut remap) = self.remap {
            if remap.size != size {
                self.remap = None;
            }
        }

        let UndistortPipeline {
            undistorted,
            cropped,
            mtx,
            dist,
            remap,
            camera_entity,
        } = self;

        *camera_entity = cmds.camera_entity;

        let RemapData {
            map_x, map_y, roi, ..
        } = match remap {
            Some(remap) => remap,
            None => {
                let mut roi = Rect::default();
                let new_mtx = calib3d::get_optimal_new_camera_matrix(
                    mtx,
                    dist,
                    size,
                    0.0,
                    size,
                    Some(&mut roi),
                    false,
                )
                .context("Get optimal matrix")?;

                let new_mtx_glam =
                    Mat3A::from_cols_slice(new_mtx.data_typed().context("new_mtx as slice")?);
                cmds.camera(move |mut camera| {
                    camera.insert(CroppedCameraMatrix { mat: new_mtx_glam });
                });

                let mut map_x = Mat::default();
                let mut map_y = Mat::default();
                calib3d::init_undistort_rectify_map(
                    mtx,
                    dist,
                    &Mat::default(),
                    &new_mtx,
                    size,
                    opencv::core::CV_32F,
                    &mut map_x,
                    &mut map_y,
                )
                .context("Init rectify map")?;

                remap.insert(RemapData {
                    size,
                    map_x,
                    map_y,
                    roi,
                })
            }
        };

        imgproc::remap_def(img, undistorted, map_x, map_y, imgproc::INTER_LINEAR)
            .context("Remap")?;

        // FIXME: This clones is bad
        *cropped = undistorted.roi(*roi).context("Crop ROI")?.clone_pointee();

        Ok(cropped)
    }

    fn cleanup(self, entity_world: &mut EntityWorldMut) {
        entity_world.world_scope(|world| {
            let Ok(mut camera) = world.get_entity_mut(self.camera_entity) else {
                return;
            };
            camera.remove::<CroppedCameraMatrix>();
        });
    }
}

impl FromWorldEntity for UndistortPipeline {
    fn from(world: &mut World, camera: Entity) -> anyhow::Result<Self>
    where
        Self: Sized,
    {
        let calib = world
            .get::<CameraCalibration>(camera)
            .context("Camera entity must exist and have calib")?;

        let mtx = Mat::from_slice_2d(&calib.camera_matrix.to_cols_array_2d())
            .context("Mat from camera matrix")?;
        let dist = Mat::from_slice_2d(&[&calib.distortion_coefficients])
            .context("Mat from dist coeffs")?;

        Ok(Self {
            undistorted: Mat::default(),
            cropped: Mat::default(),
            mtx,
            dist,
            remap: None,
            camera_entity: camera,
        })
    }
}
