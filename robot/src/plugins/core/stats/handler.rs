use std::marker::PhantomData;

use bevy::prelude::*;

use crate::plugins::core::robot::LocalRobotMarker;

use super::Statistic;

pub struct HandlerPlugin<Stat, const LIFETIME: bool> {
    pub phantom: PhantomData<Stat>,
}

// FIXME: I got fedup with the generics
#[macro_export]
macro_rules! make_handler_plugin {
    ($stat:ty) => {
        impl<const LIFETIME: bool> Plugin for HandlerPlugin<$stat, LIFETIME> {
            fn build(&self, app: &mut App) {
                app.add_systems(PostUpdate, update_statistic::<$stat, LIFETIME>);
            }
        }
    };
}

#[derive(Component)]
pub struct StatisticContainer<Stat, const LIFETIME: bool>(pub Stat);

pub fn update_statistic<Stat: Statistic + Send + Sync + 'static, const LIFETIME: bool>(
    mut cmds: Commands,
    stat: Query<(Entity, Option<&StatisticContainer<Stat, LIFETIME>>), With<LocalRobotMarker>>,
    query: Stat::Q<'_, '_>,
) {
    let (robot, stat) = stat.single();
    let Some(stat) = Stat::update(stat.map(|it| &it.0), query) else {
        return;
    };
    cmds.entity(robot)
        .insert(StatisticContainer::<Stat, LIFETIME>(stat));
}
