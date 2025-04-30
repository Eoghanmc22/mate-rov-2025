use std::marker::PhantomData;

use bevy::prelude::*;
use opencv::core::Mat;

use super::{Pipeline, PipelineCallbacks};

pub struct CopyToEcsPipeline<T>(PhantomData<T>);

impl<T> Default for CopyToEcsPipeline<T> {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<T> Pipeline for CopyToEcsPipeline<T>
where
    for<'a> T: Bundle + From<&'a Mat>,
{
    type Input = ();

    fn collect_inputs(world: &World, entity: &EntityRef) -> Self::Input {
        ()
    }

    fn process<'b, 'a: 'b>(
        &'a mut self,
        cmds: &mut PipelineCallbacks,
        data: &Self::Input,
        img: &'b mut Mat,
    ) -> anyhow::Result<&'b mut Mat> {
        cmds.should_end();

        let bundle = T::from(img);
        cmds.world(|world| {
            world.spawn(bundle);
        });

        Ok(img)
    }

    fn cleanup(self, entity_world: &mut EntityWorldMut) {}
}
