use core::panic;

use bevy::{
    ecs::{event::Event, world::World},
    reflect::{FromReflect, FromType, PartialReflect},
};

#[derive(Clone)]
pub struct ReflectEvent {
    pub send: fn(&mut World, &dyn PartialReflect),
}

impl ReflectEvent {
    pub fn send(&self, world: &mut World, event: &dyn PartialReflect) {
        (self.send)(world, event);
    }
}

impl<E: Event + PartialReflect + FromReflect> FromType<E> for ReflectEvent {
    fn from_type() -> Self {
        ReflectEvent {
            send: |world, reflected_event| {
                let Some(event) = E::from_reflect(reflected_event) else {
                    panic!("Could not create concrete event from reflected event!");
                };
                world.send_event(event);
            },
        }
    }
}
