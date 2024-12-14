use std::time::Duration;

use bevy::{app::Plugin, math::NormedVectorSpace, prelude::*};
use common::components::{ActualMovement, Orientation, Robot};

use crate::{
    trajectory::{CurrentPose, Pose},
    waterlinked::{self, WaterlinkedAngleOffset},
};

const THRUST_THRESHOLD: f32 = 2.5;
const VELOCITY_THRESHOLD: f32 = 0.5;
const TIME_EPSILON: Duration = Duration::from_millis(300);
const LATAENCY_ALPHA: f32 = 0.35;
const LEARN_ALPHA: f32 = 0.1;
const LEARN_MAX_DIFFERENCE: f32 = 15.0f32.to_radians();

const USE_REFERENCE_FRAME_LEARNING: bool = true;

pub struct LearnReferenceFramePlugin;

impl Plugin for LearnReferenceFramePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LearningState>()
            .add_systems(
                Update,
                (
                    update_state.after(waterlinked::pose_updater),
                    latency_estimator.after(update_state),
                    learn_reference_frame.after(latency_estimator),
                ),
            )
            .add_event::<FullStateChangeNotif>();
    }
}

#[derive(Event)]
struct FullStateChangeNotif;

#[derive(Debug, Default, PartialEq, Eq)]
enum VelocityState {
    Moving,
    #[default]
    Still,
}

#[derive(Debug, Default)]
struct VelocityStateObservation {
    state: VelocityState,
    start_time: Duration,
    last_time: Duration,
    just_changed: bool,
}

impl VelocityStateObservation {
    fn observe(&mut self, state: VelocityState, time: Duration) {
        let state_changed = state != self.state;
        self.state = state;
        self.just_changed = state_changed;
        if state_changed {
            self.start_time = time;
        }
        self.last_time = time;
    }
}

#[derive(Resource)]
struct LearningState {
    robot_state: VelocityStateObservation,
    waterlinked_state: VelocityStateObservation,

    // Seconds
    position_latency_estimate: Duration,

    // (velo, thrust, Bevy Time<Real>)
    robot_velocity_buffer: Vec<(Vec2, Vec2, Duration)>,
}

impl Default for LearningState {
    fn default() -> Self {
        Self {
            robot_state: Default::default(),
            waterlinked_state: Default::default(),
            position_latency_estimate: Duration::from_secs_f32(0.5),
            robot_velocity_buffer: Default::default(),
        }
    }
}

fn latency_estimator(mut state: ResMut<LearningState>) {
    if state.waterlinked_state.state == VelocityState::Moving
        && state.robot_state.state == VelocityState::Still
        && state
            .waterlinked_state
            .last_time
            .saturating_sub(state.robot_state.start_time)
            > TIME_EPSILON + state.position_latency_estimate
    {
        warn!("State miss match, waterlinked reports moving while robot is meant to be still");
        return;
    }

    if state.waterlinked_state.state == state.robot_state.state
        && state.waterlinked_state.just_changed
    {
        let latency_secs = (state
            .waterlinked_state
            .start_time
            .saturating_sub(state.robot_state.start_time))
        .as_secs_f32();
        let old_latency_secs = state.position_latency_estimate.as_secs_f32();

        let new_latency_estimate =
            latency_secs * LATAENCY_ALPHA + old_latency_secs * (1.0 - LATAENCY_ALPHA);

        info!("Updated latency estimate: observed: {latency_secs:.2}, old: {old_latency_secs:.2}, new: {new_latency_estimate:.2}");

        state.position_latency_estimate = Duration::from_secs_f32(new_latency_estimate);
    }
}

fn update_state(
    mut last_waterlinked_pose: Local<Option<(Pose, Duration)>>,
    mut state: ResMut<LearningState>,
    time: Res<Time<Real>>,
    query: Query<(&CurrentPose, &ActualMovement, &Orientation), With<Robot>>,
    mut events: EventWriter<FullStateChangeNotif>,
) {
    let Ok((CurrentPose(pose), ActualMovement(movement), Orientation(quat))) = query.get_single()
    else {
        *state = LearningState::default();
        *last_waterlinked_pose = None;
        return;
    };

    let current_time = time.elapsed();

    let world_force = (*quat * movement.force).xy();
    if world_force.norm_squared() > THRUST_THRESHOLD * THRUST_THRESHOLD {
        state
            .robot_state
            .observe(VelocityState::Moving, current_time);
    } else {
        state
            .robot_state
            .observe(VelocityState::Still, current_time);
    }

    if let Some((last_pose, last_pose_time)) = *last_waterlinked_pose {
        if pose.position == last_pose.position {
            return;
        }

        let delta_time = current_time.as_secs_f32() - last_pose_time.as_secs_f32();
        let velocity = (pose.position.xy() - last_pose.position.xy()) / delta_time;

        let latency_estimate = state.position_latency_estimate;
        state
            .robot_velocity_buffer
            .push((velocity, world_force, current_time));
        state
            .robot_velocity_buffer
            .retain(|(_velo, _thrust, time)| {
                *time
                    > current_time
                        .saturating_sub(latency_estimate)
                        .saturating_sub(TIME_EPSILON)
            });

        if velocity.norm_squared() > VELOCITY_THRESHOLD * VELOCITY_THRESHOLD {
            state
                .waterlinked_state
                .observe(VelocityState::Moving, current_time);
        } else {
            state
                .waterlinked_state
                .observe(VelocityState::Still, current_time);
        }

        events.send(FullStateChangeNotif);
    }

    *last_waterlinked_pose = Some((*pose, current_time));
}

fn learn_reference_frame(
    time: Res<Time<Real>>,
    state: Res<LearningState>,
    mut reference_angle: ResMut<WaterlinkedAngleOffset>,
    mut events: EventReader<FullStateChangeNotif>,
) {
    if events.is_empty() {
        return;
    }
    events.clear();

    let current_time = time.elapsed();
    if !(state.robot_state.state == VelocityState::Moving
        && state.waterlinked_state.state == VelocityState::Moving
        && state.waterlinked_state.start_time + state.position_latency_estimate < current_time)
    {
        return;
    }

    let thrust = state.robot_velocity_buffer.last();
    let velo_idx = state.robot_velocity_buffer.partition_point(|(_, _, time)| {
        *time < current_time.saturating_sub(state.position_latency_estimate)
    });
    let velo = state.robot_velocity_buffer.get(velo_idx);

    if let (Some((velo, _, _)), Some((_, thrust, _))) = (velo, thrust) {
        // need angle such that dir(velo) + angle = dir(thrust)
        let observed_rotation = Quat::from_rotation_arc_2d(*velo, *thrust);

        if USE_REFERENCE_FRAME_LEARNING {
            let old_ref_angle = reference_angle.0;

            if Quat::angle_between(reference_angle.0, observed_rotation).abs()
                < LEARN_MAX_DIFFERENCE
            {
                reference_angle.0 = reference_angle.0.slerp(observed_rotation, LEARN_ALPHA);
            } else {
                reference_angle.0 = observed_rotation;
            }

            info!(
                "Rotated reference frame by: {:.2} deg",
                Quat::angle_between(old_ref_angle, reference_angle.0).to_degrees()
            )
        }
    }
}
