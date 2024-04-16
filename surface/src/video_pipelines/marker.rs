use anyhow::Context;
use bevy::{
    app::{App, Plugin},
    core,
    prelude::{EntityRef, EntityWorldMut, World},
};
use opencv::{
    core::{Point, Scalar},
    imgproc,
    prelude::*,
};

use crate::video_pipelines::{AppPipelineExt, Pipeline, PipelineCallbacks};

pub struct MarkerPipelinePlugin;

impl Plugin for MarkerPipelinePlugin {
    fn build(&self, app: &mut App) {
        app.register_video_pipeline::<MarkerPipeline>();
    }
}

#[derive(Default)]
struct MarkerPipeline {
    edges: Mat,
}

impl Pipeline for MarkerPipeline {
    const NAME: &'static str = "Marker Pipeline";

    type Input = ();

    fn collect_inputs(world: &World, entity: &EntityRef) -> Self::Input {
        // No-op
    }

    fn process<'b, 'a: 'b>(
        &'a mut self,
        cmds: PipelineCallbacks,
        data: &Self::Input,
        img: &'b mut Mat,
    ) -> anyhow::Result<&'b Mat> {
        opencv::imgproc::draw_marker(
            img,
            Point::new(720, 480),
            Scalar::new(0.5, 1.0, 0.75, 1.0),
            imgproc::MARKER_CROSS,
            4,
            1,
            imgproc::LINE_8,
        )
        .context("Draw marker")?;
        Ok(img)
    }

    fn cleanup(entity_world: &mut EntityWorldMut) {
        // No-op
    }
}