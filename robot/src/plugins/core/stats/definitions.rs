use std::time::Duration;

use bevy::{ecs::system::SystemParam, prelude::*};
use common::{
    components::{Armed, CurrentDraw, DepthMeasurement, MeasuredVoltage, Robot},
    ecs_sync::ForignOwned,
    types::units::{Amperes, Meters, Volts},
};
use serde::{Deserialize, Serialize};

use crate::plugins::{core::robot::LocalRobotMarker, monitor::voltage::BrownedOut};

use super::Statistic;

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct PowerOnTimeStat {
    pub time: Duration,
}

impl Statistic for PowerOnTimeStat {
    type Q<'world, 'state> = Res<'world, Time<Real>>;

    fn update(this: Option<&Self>, time: Res<Time<Real>>) -> Option<Self> {
        let mut this = this.cloned().unwrap_or(PowerOnTimeStat {
            time: Duration::ZERO,
        });

        this.time += time.delta();

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct DriveTimeStat {
    pub time: Duration,
}

impl Statistic for DriveTimeStat {
    // TODO: Make this a single in bevy 0.16
    type Q<'world, 'state> = (
        Res<'world, Time<Real>>,
        Query<'world, 'state, &'static Armed, With<LocalRobotMarker>>,
    );

    fn update(this: Option<&Self>, (time, robot): Self::Q<'_, '_>) -> Option<Self> {
        let armed = robot.get_single().ok()?;

        let mut this = this.cloned().unwrap_or(DriveTimeStat {
            time: Duration::ZERO,
        });

        if let Armed::Armed = *armed {
            this.time += time.delta();
        }

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct MaximumDepthStat {
    pub depth: Meters,
}

impl Statistic for MaximumDepthStat {
    type Q<'world, 'state> =
        Query<'world, 'state, Option<&'static DepthMeasurement>, With<LocalRobotMarker>>;

    fn update(this: Option<&Self>, robot: Self::Q<'_, '_>) -> Option<Self> {
        let depth = robot.get_single().ok()?;

        let mut this = this.cloned().unwrap_or(MaximumDepthStat {
            depth: Meters::ZERO,
        });

        if let Some(depth) = depth {
            this.depth.0 = this.depth.0.max(depth.depth.0);
        }

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct BrownOutCounterStat {
    pub count: u32,
}

impl Statistic for BrownOutCounterStat {
    // TODO: Make this a single in bevy 0.16
    type Q<'world, 'state> =
        Query<'world, 'state, Option<&'static BrownedOut>, With<LocalRobotMarker>>;

    fn update(this: Option<&Self>, robot: Self::Q<'_, '_>) -> Option<Self> {
        let browned_out = robot.get_single().ok()?;

        let mut this = this.cloned().unwrap_or(BrownOutCounterStat { count: 0 });

        if browned_out.is_some() {
            this.count += 1;
        }

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct BrownOutTimeStat {
    pub time: Duration,
}

impl Statistic for BrownOutTimeStat {
    // TODO: Make this a single in bevy 0.16
    type Q<'world, 'state> = (
        Res<'world, Time<Real>>,
        Query<'world, 'state, Option<&'static BrownedOut>, With<LocalRobotMarker>>,
    );

    fn update(this: Option<&Self>, (time, robot): Self::Q<'_, '_>) -> Option<Self> {
        let browned_out = robot.get_single().ok()?;

        let mut this = this.cloned().unwrap_or(BrownOutTimeStat {
            time: Duration::ZERO,
        });

        if browned_out.is_some() {
            this.time += time.delta();
        }

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct MaximumVoltageStat {
    /// Maximum voltage seen since boot
    pub voltage: Volts,
    /// The current draw when that occurred
    pub current: Amperes,
}

impl Statistic for MaximumVoltageStat {
    type Q<'world, 'state> = Query<
        'world,
        'state,
        (&'static MeasuredVoltage, &'static CurrentDraw),
        With<LocalRobotMarker>,
    >;

    fn update(this: Option<&Self>, robot: Self::Q<'_, '_>) -> Option<Self> {
        let (&MeasuredVoltage(voltage), &CurrentDraw(current)) = robot.get_single().ok()?;

        let mut this = this
            .cloned()
            .unwrap_or(MaximumVoltageStat { voltage, current });

        if voltage > this.voltage {
            this.voltage = voltage;
            this.current = current;
        }

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct MinimumVoltageStat {
    /// Minimum voltage seen since boot
    pub voltage: Volts,
    /// The current draw when that occurred
    pub current: Amperes,
}

impl Statistic for MinimumVoltageStat {
    type Q<'world, 'state> = Query<
        'world,
        'state,
        (&'static MeasuredVoltage, &'static CurrentDraw),
        With<LocalRobotMarker>,
    >;

    fn update(this: Option<&Self>, robot: Self::Q<'_, '_>) -> Option<Self> {
        let (&MeasuredVoltage(voltage), &CurrentDraw(current)) = robot.get_single().ok()?;

        let mut this = this
            .cloned()
            .unwrap_or(MinimumVoltageStat { voltage, current });

        if voltage < this.voltage {
            this.voltage = voltage;
            this.current = current;
        }

        Some(this)
    }
}

#[derive(Serialize, Deserialize, Reflect, Debug, Copy, Clone, PartialEq, Default)]
pub struct MaximumCurrentStat {
    /// Maximum current seen since boot
    pub current: Amperes,
    /// The voltage when that occurred
    pub voltage: Volts,
}

impl Statistic for MaximumCurrentStat {
    type Q<'world, 'state> = Query<
        'world,
        'state,
        (&'static MeasuredVoltage, &'static CurrentDraw),
        With<LocalRobotMarker>,
    >;

    fn update(this: Option<&Self>, robot: Self::Q<'_, '_>) -> Option<Self> {
        let (&MeasuredVoltage(voltage), &CurrentDraw(current)) = robot.get_single().ok()?;

        let mut this = this
            .cloned()
            .unwrap_or(MaximumCurrentStat { voltage, current });

        if current > this.current {
            this.voltage = voltage;
            this.current = current;
        }

        Some(this)
    }
}
