pub mod definitions;
#[macro_use]
pub mod handler;

use crate::plugins::core::stats::handler::StatisticContainer;
use std::{fs, marker::PhantomData, time::Duration};

use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::{
    components::Armed,
    sync::Peer,
    types::units::{Amperes, Meters, Volts},
};
use definitions::*;
use handler::{update_statistic, HandlerPlugin};
use serde::{de::DeserializeOwned, ser::SerializeMap, Deserialize, Serialize};

use super::robot::{LocalRobot, LocalRobotMarker};

pub struct StatisticsPlugin;

impl Plugin for StatisticsPlugin {
    fn build(&self, app: &mut App) {
        register_handler_plugins(app);

        app.add_systems(Startup, setup).add_systems(Update, save);
    }
}

pub fn setup(mut cmds: Commands, robot: Res<LocalRobot>) {
    let res = fs::read_to_string("stats.toml");
    if let Ok(stats) = res {
        let res = toml::from_str(&stats);
        let Ok(stats): Result<LifetimeStatistics, _> = res else {
            error!("Could not load past stats: {res:?}");
            return;
        };

        cmds.entity(robot.entity).insert(stats.to_bundle());
    }
}

pub fn save(
    mut timer: Local<Option<Timer>>,
    time: Res<Time<Real>>,
    query: Query<LifetimeTupleOptionRef, With<LocalRobotMarker>>,
) {
    let timer = timer
        .get_or_insert_with(|| Timer::new(Duration::from_secs_f32(20.0), TimerMode::Repeating));
    timer.tick(time.delta());

    if timer.just_finished() {
        let stats = query.single();
        let stats = LifetimeStatistics::from_bundle_option_ref(stats);
        let Ok(str) = toml::to_string_pretty(&stats) else {
            // TODO: Print the actual error
            error!("Could not serialize stats");
            return;
        };
        // TODO: Do in non blocking manner
        let res = fs::write("stats.toml", &str);
        if let Err(err) = res {
            error!("Could not write past stats: {err:?}");
        }
    }
}

pub trait Statistic: Serialize + DeserializeOwned {
    type Q<'world, 'state>: SystemParam;

    fn update(this: Option<&Self>, query: Self::Q<'_, '_>) -> Option<Self>;
}

macro_rules! configure_statistics {
    ($($stat:ident),*) => {
        $(
            make_handler_plugin!($stat);
        )*

        fn register_handler_plugins(app: &mut App) {
            $(
                app.add_plugins(HandlerPlugin::<$stat, true> { phantom: PhantomData });
                app.add_plugins(HandlerPlugin::<$stat, false> { phantom: PhantomData });
            )*
        }

        #[derive(Serialize, Deserialize, Debug)]
        #[allow(non_snake_case)]
        #[serde(rename_all = "snake_case")]
        pub struct LifetimeStatistics{
            $(
                #[serde(default)]
                $stat: $stat
            ),*
        }

        pub type LifetimeTuple = ($(StatisticContainer::<$stat, true>),*);
        pub type LifetimeTupleOptionRef<'a> = ($(Option<&'a StatisticContainer::<$stat, true>>),*);
        pub type RuntimeTuple = ($(StatisticContainer::<$stat, false>),*);
        pub type RuntimeTupleOptionRef<'a> = ($(Option<&'a StatisticContainer::<$stat, false>>),*);

        #[allow(non_snake_case)]
        impl LifetimeStatistics {
            pub fn to_bundle(self) -> LifetimeTuple {
                (
                    ($(StatisticContainer(self.$stat)),*)
                )
            }

            pub fn from_bundle_option_ref(($($stat),*): LifetimeTupleOptionRef) -> Self {
                Self {
                    $($stat: $stat.map(|it| it.0).unwrap_or_default()),*
                }
            }
        }
    };
}

configure_statistics! {
    PowerOnTimeStat,
    DriveTimeStat,
    MaximumDepthStat,
    BrownOutCounterStat,
    BrownOutTimeStat,
    MaximumVoltageStat,
    MinimumVoltageStat,
    MaximumCurrentStat
}
