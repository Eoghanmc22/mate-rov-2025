use std::time::Duration;

use bevy::{
    app::{App, Plugin, Update},
    prelude::*,
};
use bevy_tokio_tasks::TokioTasksRuntime;
use common::components::{Magnetic, Robot};
use serde::{Deserialize, Serialize};
use tokio::{fs::File, io::AsyncWriteExt};

pub struct LearnCompassPlugin;

impl Plugin for LearnCompassPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MagneticData>();
        app.add_event::<ResetMagneticLog>();
        app.add_systems(Update, (log_compass, reset_log.after(save_log), save_log));
    }
}

#[derive(Event, Default)]
pub struct ResetMagneticLog;

#[derive(Resource, Debug, Default, Clone, Serialize, Deserialize)]
struct MagneticData {
    min_x: f32,
    max_x: f32,

    min_y: f32,
    max_y: f32,

    min_z: f32,
    max_z: f32,
}

fn log_compass(mut log: ResMut<MagneticData>, robot: Query<&Magnetic, With<Robot>>) {
    for magnetic in robot.iter() {
        log.min_x = log.min_x.min(magnetic.0.mag_x.0);
        log.max_x = log.max_x.max(magnetic.0.mag_x.0);

        log.min_y = log.min_y.min(magnetic.0.mag_y.0);
        log.max_y = log.max_y.max(magnetic.0.mag_y.0);

        log.min_z = log.min_z.min(magnetic.0.mag_z.0);
        log.max_z = log.max_z.max(magnetic.0.mag_z.0);
    }
}

fn reset_log(mut cmds: Commands, mut events: EventReader<ResetMagneticLog>) {
    for _ in events.read() {
        cmds.insert_resource(MagneticData::default());
        println!("Reset mag log");
    }
}

fn save_log(
    mut timer: Local<Option<Timer>>,
    log: Res<MagneticData>,
    time: Res<Time<Real>>,
    runtime: Res<TokioTasksRuntime>,
    mut events: EventWriter<ResetMagneticLog>,
) {
    let timer =
        timer.get_or_insert_with(|| Timer::new(Duration::from_secs(5 * 60), TimerMode::Repeating));

    timer.tick(time.delta());

    if timer.finished() {
        let log = log.clone();
        runtime.spawn_background_task(|_| async move {
            let Ok(json) = serde_json::to_vec(&log) else {
                error!("Couldnt serialize mag log");
                return;
            };

            let Ok(mut file) = File::options().append(true).open("mag_learn.log").await else {
                error!("Couldnt open mag log file");
                return;
            };

            let Ok(()) = file.write_all(&json).await else {
                error!("Couldnt append to mag log");
                return;
            };
        });

        events.send(ResetMagneticLog);
    }
}
