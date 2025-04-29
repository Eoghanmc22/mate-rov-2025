use std::{borrow::Cow, mem};

use ahash::HashSet;
use bevy::{
    math::{vec3a, Vec3A},
    prelude::*,
};
use common::{
    bundles::MovementContributionBundle,
    components::{
        Armed, CameraInputRotation, DepthMeasurement, DepthTarget, GenericMotorId,
        MotorContribution, Motors, MovementAxisMaximums, MovementContribution, Orientation,
        OrientationTarget, Robot, RobotId, Thrusters,
    },
    ecs_sync::{NetId, Replicate},
    events::ResetServo,
    types::units::Meters,
};
use egui::TextBuffer;
use leafwing_input_manager::{
    action_state::ActionState, input_map::InputMap, plugin::InputManagerPlugin, Actionlike,
    InputManagerBundle,
};
use motor_math::{glam::MovementGlam, solve::reverse::Axis, Movement};

use crate::{photosphere::TakePhotoSphereImage, video_display_2d_master::VideoMasterMarker};

// TODO(low): Handle multiple gamepads better
pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<InputInterpolation>()
            .register_type::<SelectedServo>();

        app.add_plugins(InputManagerPlugin::<Action>::default())
            .add_systems(
                Update,
                (
                    attach_to_new_robots,
                    handle_disconnected_robots,
                    movement,
                    arm,
                    depth_hold,
                    leveling,
                    trim_orientation,
                    trim_depth,
                    servos,
                    robot_mode,
                    take_photo_sphere_image,
                    // switch_pitch_roll,
                ),
            );
    }
}

#[derive(Component, Debug, Clone, Default, Reflect)]
pub struct SelectedServo {
    pub servo: Option<(GenericMotorId, Cow<'static, str>)>,
}

#[derive(Component, Debug, Clone, Copy, Reflect, PartialEq)]
pub struct InputInterpolation {
    depth_mps: f32,
    trim_dps: Vec3A,
    servo_rate: f32,

    power: f32,
    scale: f32,

    translate_gain: Vec3A,
    translate_gain_depth_hold: Vec3A,
    torque_gain: Vec3A,
    torque_gain_stabalize: Vec3A,
}

impl InputInterpolation {
    pub fn interpolate_input(&self, input: f32) -> f32 {
        input.powf(self.power).copysign(input) * self.scale
    }

    pub const fn normal() -> Self {
        Self {
            depth_mps: 0.3,
            trim_dps: vec3a(35.0, 35.0, 100.0),
            servo_rate: 1.5,
            power: 3.0,
            scale: 0.8,
            translate_gain: vec3a(1.0, 1.0, 1.0),
            translate_gain_depth_hold: vec3a(1.0, 1.0, 0.1),
            torque_gain: vec3a(1.0, 1.0, 0.5),
            torque_gain_stabalize: vec3a(0.0, 0.0, 0.0),
        }
    }

    pub const fn precision() -> Self {
        Self {
            depth_mps: 0.2,
            trim_dps: vec3a(25.0, 25.0, 60.0),
            servo_rate: 1.0,
            power: 3.0,
            scale: 0.1,
            translate_gain: vec3a(1.0, 1.0, 1.0),
            translate_gain_depth_hold: vec3a(2.0, 1.0, 0.0),
            torque_gain: vec3a(1.0, 1.0, 0.5),
            torque_gain_stabalize: vec3a(0.0, 0.0, 0.0),
        }
    }
}

#[derive(Actionlike, PartialEq, Eq, Hash, Clone, Copy, Debug, Reflect)]
pub enum Action {
    Arm,
    Disarm,

    // IncreaseGain,
    // DecreaseGain,
    // ResetGain,
    ToggleDepthHold,
    ToggleLeveling(LevelingType),

    ToggleRobotMode,

    #[actionlike(Axis)]
    Surge,
    #[actionlike(Axis)]
    SurgeInverted,
    #[actionlike(Axis)]
    Heave,
    #[actionlike(Axis)]
    HeaveInverted,
    #[actionlike(Axis)]
    Sway,
    #[actionlike(Axis)]
    SwayInverted,

    // #[actionlike(Axis)]
    Pitch,
    // #[actionlike(Axis)]
    PitchInverted,
    // #[actionlike(Axis)]
    Roll,
    // #[actionlike(Axis)]
    RollInverted,
    #[actionlike(Axis)]
    Yaw,
    #[actionlike(Axis)]
    YawInverted,
    // HoldAxis,
    Servo,
    ServoCenter,
    ServoInverted,
    SwitchServo,
    SwitchServoInverted,
    SelectImportantServo,

    SwitchPitchRoll,

    TakePhotoSphereImage,
}

#[derive(Actionlike, PartialEq, Eq, Hash, Clone, Copy, Debug, Reflect, Default)]
pub enum LevelingType {
    #[default]
    Upright,
    Inverted,
}

#[derive(Component)]
pub struct InputMarker;

fn attach_to_new_robots(mut cmds: Commands, new_robots: Query<(&NetId, &Name), Added<Robot>>) {
    for (robot, name) in &new_robots {
        let mut input_map = InputMap::default();

        input_map.insert(Action::Disarm, GamepadButton::Select);
        input_map.insert(Action::Arm, GamepadButton::Start);

        input_map.insert(Action::Disarm, KeyCode::Space);
        input_map.insert(Action::Arm, KeyCode::Enter);

        input_map.insert(
            Action::ToggleLeveling(LevelingType::Upright),
            GamepadButton::North,
        );
        input_map.insert(
            Action::ToggleLeveling(LevelingType::Inverted),
            GamepadButton::South,
        );
        input_map.insert(Action::ToggleDepthHold, GamepadButton::East);
        // input_map.insert(Action::ToggleDepthHold, GamepadButton::North);
        // input_map.insert(Action::ToggleDepthHold, GamepadButton::South);
        // input_map.insert(Action::SwitchPitchRoll, GamepadButton::West);
        input_map.insert(Action::TakePhotoSphereImage, GamepadButton::West);

        input_map.insert_axis(Action::Yaw, GamepadAxis::LeftStickX);
        input_map.insert_axis(Action::Surge, GamepadAxis::LeftStickY);

        input_map.insert_axis(Action::Sway, GamepadAxis::RightStickX);
        input_map.insert_axis(Action::Heave, GamepadAxis::RightStickY);

        input_map.insert(Action::ServoInverted, GamepadButton::LeftTrigger);
        input_map.insert(Action::Servo, GamepadButton::RightTrigger);
        // input_map.insert(Action::ServoInverted, GamepadButton::RightTrigger2);
        // input_map.insert(Action::Servo, GamepadButton::LeftTrigger2);

        // input_map.insert(Action::Pitch, GamepadButton::RightTrigger);
        // input_map.insert(Action::PitchInverted, GamepadButton::LeftTrigger);

        // input_map.insert(Action::Roll, GamepadButton::RightTrigger2);
        // input_map.insert(Action::RollInverted, GamepadButton::LeftTrigger2);
        input_map.insert(Action::Pitch, GamepadButton::RightTrigger2);
        input_map.insert(Action::PitchInverted, GamepadButton::LeftTrigger2);

        input_map.insert(Action::ServoCenter, GamepadButton::DPadUp);
        // input_map.insert(Action::Servo, GamepadButton::DPadRight);
        // input_map.insert(Action::ServoInverted, GamepadButton::DPadLeft);
        input_map.insert(Action::SwitchServo, GamepadButton::DPadRight);
        input_map.insert(Action::SwitchServoInverted, GamepadButton::DPadLeft);
        // input_map.insert(Action::SelectImportantServo, GamepadButton::DPadDown);
        input_map.insert(Action::ToggleRobotMode, GamepadButton::DPadDown);

        input_map.insert(Action::ToggleRobotMode, GamepadButton::Mode);
        // input_map.insert(Action::ToggleRobotMode, GamepadButton::West);

        // input_map.insert(
        //     Action::Yaw,
        //     SingleAxis::symmetric(GamepadAxis::LeftStickX, 0.05),
        // );
        // input_map.insert(
        //     Action::Pitch,
        //     SingleAxis::symmetric(GamepadAxis::LeftStickY, 0.05),
        // );
        //
        // input_map.insert(
        //     Action::Sway,
        //     SingleAxis::symmetric(GamepadAxis::RightStickX, 0.05),
        // );
        // input_map.insert(
        //     Action::Heave,
        //     SingleAxis::symmetric(GamepadAxis::RightStickY, 0.05),
        // );
        //
        // input_map.insert(Action::Roll, GamepadButton::RightTrigger);
        // input_map.insert(Action::RollInverted, GamepadButton::LeftTrigger);
        //
        // input_map.insert(Action::Surge, GamepadButton::RightTrigger2);
        // input_map.insert(Action::SurgeInverted, GamepadButton::LeftTrigger2);

        cmds.spawn((
            SelectedServo::default(),
            InputManagerBundle::<Action> {
                // Stores "which actions are currently pressed"
                action_state: ActionState::default(),
                // Describes how to convert from player inputs into those actions
                input_map,
            },
            MovementContributionBundle {
                name: Name::new(format!("HID {name}")),
                contribution: MovementContribution(MovementGlam::default()),
                robot: RobotId(*robot),
            },
            MotorContribution(Default::default()),
            InputInterpolation::normal(),
            InputMarker,
            Replicate,
        ));
    }
}

fn handle_disconnected_robots(
    mut cmds: Commands,
    robots: Query<&NetId, With<Robot>>,
    inputs: Query<(Entity, &RobotId), With<InputMarker>>,
    mut removed_robots: RemovedComponents<Robot>,
) {
    for _robot in removed_robots.read() {
        let robots: HashSet<NetId> = robots.iter().copied().collect();

        inputs
            .iter()
            .filter(|(_, &RobotId(robot))| !robots.contains(&robot))
            .for_each(|(entity, _)| cmds.entity(entity).despawn());
    }
}

// TODO(mid): Remap sticks to square. See http://theinstructionlimit.com/squaring-the-thumbsticks
fn movement(
    mut cmds: Commands,
    inputs: Query<(Entity, &RobotId, &ActionState<Action>, &InputInterpolation), With<InputMarker>>,
    robots: Query<
        (
            &MovementAxisMaximums,
            Option<&DepthTarget>,
            Option<&Orientation>,
            Option<&OrientationTarget>,
            &RobotId,
        ),
        With<Robot>,
    >,
    selected_camera: Query<(&CameraInputRotation, &RobotId), With<VideoMasterMarker>>,
) {
    for (entity, robot, action_state, interpolation) in &inputs {
        let Some((
            MovementAxisMaximums(maximums),
            depth_target,
            orientation,
            orientation_target,
            _,
        )) = robots
            .iter()
            .find(|(_, _, _, _, robot_id)| robot_id.0 == robot.0)
        else {
            error!("Could not find robot for input");

            continue;
        };

        let input_rotation = selected_camera
            .iter()
            .filter(|(_, robot_id)| robot_id.0 == robot.0)
            .map(|(it, _)| it.0)
            .next()
            .unwrap_or_default();

        let translate_gain = if depth_target.is_some() {
            interpolation.translate_gain_depth_hold
        } else {
            interpolation.translate_gain
        };

        let torque_gain = if orientation_target.is_some() {
            interpolation.torque_gain_stabalize
        } else {
            interpolation.torque_gain
        };

        let force = vec3a(
            interpolation.interpolate_input(
                action_state.value(&Action::Sway) - action_state.value(&Action::SwayInverted),
            ),
            interpolation.interpolate_input(
                action_state.value(&Action::Surge) - action_state.value(&Action::SurgeInverted),
            ),
            interpolation.interpolate_input(
                action_state.value(&Action::Heave) - action_state.value(&Action::HeaveInverted),
            ),
        );
        let force = input_rotation * force;
        let force = force
            * vec3a(
                maximums[&Axis::X].0,
                maximums[&Axis::Y].0,
                maximums[&Axis::Z].0,
            )
            * translate_gain;

        let torque = vec3a(
            interpolation.interpolate_input(
                action_state.button_value(&Action::Pitch)
                    - action_state.button_value(&Action::PitchInverted),
            ),
            interpolation.interpolate_input(
                action_state.button_value(&Action::Roll)
                    - action_state.button_value(&Action::RollInverted),
            ),
            interpolation.interpolate_input(
                -(action_state.value(&Action::Yaw) - action_state.value(&Action::YawInverted)),
            ),
        );
        let torque = input_rotation * torque;
        let torque = torque
            * vec3a(
                maximums[&Axis::XRot].0,
                maximums[&Axis::YRot].0,
                maximums[&Axis::ZRot].0,
            )
            * torque_gain;

        // TODO: We should never zero the z input, this should instead allow switching between
        // interperting z as local vs global
        let force = if depth_target.is_some() {
            if let Some(orientation) = orientation {
                // TODO: Validate this actually works, and make it into a helper function, also
                // used for heading display
                let mut yaw = orientation.0;
                if yaw.z.abs() * yaw.z.abs() + yaw.w.abs() * yaw.w.abs() > 0.1 {
                    yaw.x = 0.0;
                    yaw.y = 0.0;
                    yaw = yaw.normalize()
                } else {
                    yaw *= Quat::from_rotation_y(180f32.to_radians());
                    yaw.x = 0.0;
                    yaw.y = 0.0;
                    yaw = -yaw.normalize();
                    // yaw *= Quat::from_rotation_y(180f32.to_radians()).inverse();
                }

                let world_force = yaw * force;

                orientation.0.inverse() * world_force
            } else {
                force
            }
        } else {
            force
        };

        // TODO: torque vector should always be applied to act as feed forward for pid
        // let torque = if orientation_target.is_some() {
        //     Vec3A::ZERO
        // } else {
        //     vec3a(x_rot, y_rot, z_rot)
        // };

        let movement = MovementGlam { force, torque };

        cmds.entity(entity).insert(MovementContribution(movement));
    }
}

fn arm(
    mut cmds: Commands,
    inputs: Query<(&RobotId, &ActionState<Action>), With<InputMarker>>,
    robots: Query<(Entity, &RobotId), With<Robot>>,
) {
    for (robot, action_state) in &inputs {
        let disarm = action_state.just_pressed(&Action::Disarm);
        let arm = action_state.just_pressed(&Action::Arm);

        let robot = robots.iter().find(|&(_, other_robot)| robot == other_robot);

        if let Some((robot, _)) = robot {
            if disarm {
                info!("Disarming");
                cmds.entity(robot).insert(Armed::Disarmed);
            } else if arm {
                info!("Arming");
                cmds.entity(robot).insert(Armed::Armed);
            }
        } else if arm || disarm {
            warn!("No ROV attached");
        }
    }
}

fn depth_hold(
    mut cmds: Commands,
    inputs: Query<(&RobotId, &ActionState<Action>), With<InputMarker>>,
    robots: Query<(Entity, &DepthMeasurement, Option<&DepthTarget>, &RobotId), With<Robot>>,
) {
    for (robot, action_state) in &inputs {
        let toggle = action_state.just_pressed(&Action::ToggleDepthHold);

        let robot = robots
            .iter()
            .find(|&(_, _, _, other_robot)| robot == other_robot);

        if let Some((robot, depth, depth_target, _)) = robot {
            if toggle {
                match depth_target {
                    Some(_) => {
                        info!("Clear Depth Hold");
                        cmds.entity(robot).remove::<DepthTarget>();
                    }
                    None => {
                        let depth = depth.depth;

                        info!("Set Depth Hold: {:.2}", depth);
                        cmds.entity(robot).insert(DepthTarget(depth));
                    }
                }
            }
        } else if toggle {
            warn!("No ROV attached");
        }
    }
}

fn leveling(
    mut cmds: Commands,
    inputs: Query<(&RobotId, &ActionState<Action>), With<InputMarker>>,
    robots: Query<(Entity, &Orientation, Option<&OrientationTarget>, &RobotId), With<Robot>>,
) {
    for (robot, action_state) in &inputs {
        let toggle_upright =
            action_state.just_pressed(&Action::ToggleLeveling(LevelingType::Upright));
        let toggle_inverted =
            action_state.just_pressed(&Action::ToggleLeveling(LevelingType::Inverted));

        let robot = robots
            .iter()
            .find(|&(_, _, _, other_robot)| robot == other_robot);

        if let Some((robot, orientation, orientation_target, _)) = robot {
            if toggle_upright || toggle_inverted {
                let mut new_target = orientation.0;

                // Only keep yaw component
                new_target.x = 0.0;
                new_target.y = 0.0;
                let new_target = new_target.normalize();

                // Flip if inverted is selected
                let new_target = if toggle_upright {
                    new_target
                } else {
                    new_target * Quat::from_rotation_y(180f32.to_radians())
                };

                match orientation_target {
                    // FIXME: Make switching from upright to inverted easier
                    Some(_old_target) => {
                        //if old_target.0 == new_target => {
                        info!("Clear Leveling");
                        cmds.entity(robot).remove::<OrientationTarget>();
                    }
                    _ => {
                        if toggle_upright {
                            info!("Set Level Upright");
                        } else {
                            info!("Set Level Inverted");
                        }

                        cmds.entity(robot).insert(OrientationTarget(new_target));
                    }
                }
            }
        } else if toggle_upright || toggle_inverted {
            warn!("No ROV attached");
        }
    }
}

fn trim_orientation(
    mut cmds: Commands,
    inputs: Query<(&RobotId, &ActionState<Action>, &InputInterpolation), With<InputMarker>>,
    robots: Query<(Entity, &Orientation, Option<&OrientationTarget>, &RobotId), With<Robot>>,
    selected_camera: Query<(&CameraInputRotation, &RobotId), With<VideoMasterMarker>>,
    time: Res<Time<Real>>,
) {
    for (robot, action_state, interpolation) in &inputs {
        let input_rotation = selected_camera
            .iter()
            .filter(|(_, robot_id)| robot_id.0 == robot.0)
            .map(|(it, _)| it.0)
            .next()
            .unwrap_or_default();

        let torque = vec3a(
            interpolation.interpolate_input(
                action_state.button_value(&Action::Pitch)
                    - action_state.button_value(&Action::PitchInverted),
            ),
            interpolation.interpolate_input(
                action_state.button_value(&Action::Roll)
                    - action_state.button_value(&Action::RollInverted),
            ),
            interpolation.interpolate_input(
                -(action_state.value(&Action::Yaw) - action_state.value(&Action::YawInverted)),
            ),
        );
        let torque = input_rotation * torque;
        let torque = torque * interpolation.trim_dps;

        let robot = robots
            .iter()
            .find(|&(_, _, _, other_robot)| robot == other_robot);

        if let Some((robot, orientation, orientation_target, _)) = robot {
            let Some(&OrientationTarget(mut orientation_target)) = orientation_target else {
                continue;
            };

            if torque.x.abs() >= 0.05 {
                let input = torque.x * time.delta_secs();
                orientation_target = orientation_target * Quat::from_rotation_x(input.to_radians());
            }

            if torque.y.abs() >= 0.05 {
                let input = torque.y * time.delta_secs();
                orientation_target = orientation_target * Quat::from_rotation_y(input.to_radians());
            }

            if torque.z.abs() >= 0.05 {
                let input = torque.z * time.delta_secs();
                orientation_target = Quat::from_rotation_z(input.to_radians()) * orientation_target;
            }

            if torque.x.abs() >= 0.05 || torque.y.abs() >= 0.05 || torque.z.abs() >= 0.05 {
                cmds.entity(robot)
                    .insert(OrientationTarget(orientation_target));
            }
        } else if torque.x.abs() >= 0.05 || torque.y.abs() >= 0.05 || torque.z.abs() >= 0.05 {
            warn!("No ROV attached");
        }
    }
}

fn trim_depth(
    mut cmds: Commands,
    inputs: Query<(&RobotId, &ActionState<Action>, &InputInterpolation), With<InputMarker>>,
    robots: Query<(Entity, Option<&DepthTarget>, Option<&Orientation>, &RobotId), With<Robot>>,
    time: Res<Time<Real>>,
) {
    for (robot, action_state, interpolation) in &inputs {
        let z = interpolation.interpolate_input(
            action_state.value(&Action::Heave) - action_state.value(&Action::HeaveInverted),
        );

        let robot = robots
            .iter()
            .find(|&(_, _, _, other_robot)| robot == other_robot);

        if let Some((robot, depth_target, orientation, _)) = robot {
            let Some(&DepthTarget(Meters(mut depth_target))) = depth_target else {
                continue;
            };

            if z != 0.0 {
                let mut input = z * interpolation.depth_mps * time.delta_secs();

                // if let Some(orientation) = orientation {
                //     input *= (orientation.0 * Vec3A::Z).z.signum();
                // }

                // Positive should cause upward movement, ie depth should decrease
                depth_target += -input;
                if depth_target < 0.0 {
                    depth_target = 0.0;
                }
                cmds.entity(robot).insert(DepthTarget(depth_target.into()));
            }
        } else if z != 0.0 {
            warn!("No ROV attached");
        }
    }
}

fn servos(
    mut cmds: Commands,
    mut inputs: Query<
        (
            Entity,
            &RobotId,
            &ActionState<Action>,
            &InputInterpolation,
            // TODO: Make this not mut?
            &mut SelectedServo,
        ),
        With<InputMarker>,
    >,
    mut writer: EventWriter<ResetServo>,
    robots: Query<(&Motors, &RobotId), With<Robot>>,
    servos: Query<(&GenericMotorId, &Name, &RobotId)>,
) {
    for (entity, robot_id, action_state, interpolation, mut selected_servo) in &mut inputs {
        let center = action_state.just_pressed(&Action::ServoCenter);
        let switch = action_state.just_pressed(&Action::SwitchServo);
        let switch_inverted = action_state.just_pressed(&Action::SwitchServoInverted);
        let select_important = action_state.just_pressed(&Action::SelectImportantServo);
        // TODO: just why? why cant we have nice things...
        let input = action_state.button_value(&Action::Servo)
            - action_state.button_value(&Action::ServoInverted);
        // let input = action_state.value(&Action::Servo) - action_state.value(&Action::ServoInverted);

        let robot = robots
            .iter()
            .find(|&(_, other_robot_id)| robot_id == other_robot_id);

        if let Some((motors, _)) = robot {
            let offset = if switch {
                1
            } else {
                motors.ids.len().saturating_sub(1)
            };

            if select_important {
                error!("Select important servo is not implemented!");

                // if selected_servo.servo.as_ref().map(|it| it.as_str()) != Some("Claw1") {
                //     if servos.motors.iter().any(|it| it.as_str() == "Claw1") {
                //         selected_servo.servo = Some("Claw1".into());
                //     }
                // } else if servos
                //     .motors
                //     .iter()
                //     .any(|it| it.as_str() == "FrontCameraRotate")
                // {
                //     selected_servo.servo = Some("FrontCameraRotate".into());
                // }
            } else if (switch || switch_inverted || selected_servo.servo.is_none())
                && !motors.ids.is_empty()
            {
                let idx = motors
                    .ids
                    .iter()
                    .position(|it| Some(it) == selected_servo.servo.as_ref().map(|(id, _)| id))
                    .map(|it| (it + offset) % motors.ids.len())
                    .unwrap_or(0);

                let servo_id = motors.ids[idx];
                let servo_name = servos
                    .iter()
                    .filter(|(_, _, servo_robot_id)| **servo_robot_id == *robot_id)
                    .filter(|(servo_channel, _, _)| **servo_channel == servo_id)
                    .map(|(_, name, _)| Cow::from(name.as_str().to_owned()))
                    .next();
                selected_servo.servo =
                    Some((servo_id, servo_name.unwrap_or("Unknown Servo".into())));
            }

            if let Some(servo) = &selected_servo.servo {
                if center {
                    writer.send(ResetServo(servo.0));
                }

                let movement = input * interpolation.servo_rate;

                cmds.entity(entity).insert(MotorContribution(
                    vec![(servo.clone(), movement)]
                        .into_iter()
                        .map(|((id, _), output)| (id, output))
                        .collect(),
                ));
            }
        }
    }
}

fn robot_mode(
    mut inputs: Query<(&ActionState<Action>, &mut InputInterpolation), With<InputMarker>>,
) {
    for (action_state, mut interpolation) in &mut inputs {
        let toggle = action_state.just_pressed(&Action::ToggleRobotMode);

        if toggle {
            if *interpolation == InputInterpolation::normal() {
                *interpolation = InputInterpolation::precision()
            } else {
                *interpolation = InputInterpolation::normal()
            }
        }
    }
}

// FIXME: Unclear how to implement with new version
//
// fn switch_pitch_roll(
//     mut inputs: Query<(&ActionState<Action>, &mut InputMap<Action>), With<InputMarker>>,
// ) {
//     for (action_state, mut input_map) in &mut inputs {
//         let toggle = action_state.just_pressed(&Action::SwitchPitchRoll);
//
//         if toggle {
//             // Me when no proper remove api
//             let pitch = input_map.get(&Action::Pitch).clone();
//             let pitch_inverted = input_map.get(&Action::PitchInverted).clone();
//             let roll = input_map.get(&Action::Roll).clone();
//             let roll_inverted = input_map.get(&Action::RollInverted).clone();
//
//             input_map.clear_action(&Action::Pitch);
//             input_map.clear_action(&Action::PitchInverted);
//             input_map.clear_action(&Action::Roll);
//             input_map.clear_action(&Action::RollInverted);
//
//             if let Some(pitch) = pitch {
//                 for input in pitch {
//                     input_map.insert(Action::Roll, input);
//                 }
//             }
//
//             if let Some(pitch_inverted) = pitch_inverted {
//                 for input in pitch_inverted {
//                     input_map.insert(Action::RollInverted, input);
//                 }
//             }
//
//             if let Some(roll) = roll {
//                 for input in roll {
//                     input_map.insert(Action::Pitch, input);
//                 }
//             }
//
//             if let Some(roll_inverted) = roll_inverted {
//                 for input in roll_inverted {
//                     input_map.insert(Action::PitchInverted, input);
//                 }
//             }
//         }
//     }
// }

fn take_photo_sphere_image(
    mut cmds: Commands,
    inputs: Query<(&ActionState<Action>, &RobotId), With<InputMarker>>,
    robots: Query<(Entity, &RobotId), With<Robot>>,
) {
    for (input, robot_id) in inputs.iter() {
        if !input.just_pressed(&Action::TakePhotoSphereImage) {
            continue;
        }

        let Some((robot, _)) = robots
            .iter()
            .find(|&(_, other_robot_id)| robot_id == other_robot_id)
        else {
            warn!("No ROV attached");
            continue;
        };

        cmds.entity(robot).trigger(TakePhotoSphereImage);
    }
}
