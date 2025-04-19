use bevy::prelude::*;
use common::components::{CurrentDraw, MeasuredVoltage};

use crate::plugins::core::robot::LocalRobotMarker;

// TODO: Consider stopping actuators when this component is on the robot
#[derive(Component)]
pub struct BrownedOut;

pub struct VoltagePlugin;

impl Plugin for VoltagePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, check_voltage);
    }
}

fn check_voltage(
    mut cmds: Commands,
    robot: Query<(Entity, &MeasuredVoltage, &CurrentDraw), With<LocalRobotMarker>>,
) {
    for (entity, voltage, current) in &robot {
        let raw_voltage = voltage.0 .0;
        if raw_voltage < 10.0 && raw_voltage > 1.0 {
            warn!("Low Voltage: {}, {}", voltage.0, current.0);
        }
        if raw_voltage < 7.0 && raw_voltage > 1.0 {
            cmds.entity(entity).insert(BrownedOut);
        } else {
            cmds.entity(entity).remove::<BrownedOut>();
        }
    }
}
