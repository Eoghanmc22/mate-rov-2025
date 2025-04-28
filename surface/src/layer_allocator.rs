use std::sync::atomic::{AtomicUsize, Ordering};

use bevy::render::view::RenderLayers;

static RENDER_LAYER_ALLOCATOR: AtomicUsize = AtomicUsize::new(100);

pub fn next_render_layer() -> RenderLayers {
    RenderLayers::none().with(RENDER_LAYER_ALLOCATOR.fetch_add(1, Ordering::Relaxed))
}
