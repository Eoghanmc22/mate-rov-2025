use std::marker::PhantomData;

use anyhow::bail;
use bevy::prelude::*;
use opencv::core::Mat;

use crate::video_stream;

use super::{Pipeline, PipelineCallbacks};

pub struct CopyToEcsPipeline<T>(PhantomData<T>);

impl<T> Default for CopyToEcsPipeline<T> {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<T> Pipeline for CopyToEcsPipeline<T>
where
    for<'a> T: Bundle + TryFrom<CopyToEcsState<'a>>,
{
    type Input = ();

    fn collect_inputs(world: &World, entity: &EntityRef) -> Self::Input {}

    fn process<'b, 'a: 'b>(
        &'a mut self,
        cmds: &mut PipelineCallbacks,
        data: &Self::Input,
        mat: &'b mut Mat,
    ) -> anyhow::Result<&'b mut Mat> {
        cmds.should_end();

        let mut img = Image::default();
        let Ok(()) = video_stream::mat_to_image(mat, &mut img) else {
            bail!("error converting mat to image");
        };

        let pipeline_entity = cmds.pipeline_entity;
        let camera_entity = cmds.camera_entity;

        cmds.world(move |world| {
            let Ok(bundle) = T::try_from(CopyToEcsState {
                img,
                world,
                pipeline_entity,
                camera_entity,
            }) else {
                error!("Error creating bundle");
                return;
            };
            world.spawn(bundle);
        });

        Ok(mat)
    }

    fn cleanup(self, entity_world: &mut EntityWorldMut) {}
}

pub struct CopyToEcsState<'a> {
    pub img: Image,
    pub world: &'a mut World,
    pub pipeline_entity: Entity,
    pub camera_entity: Entity,
}
