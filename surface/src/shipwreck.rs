use std::thread;

use anyhow::{bail, Context};
use bevy::prelude::*;
use bevy_egui::EguiContexts;
use common::types::units::Meters;
use crossbeam::channel::{Receiver, Sender};
use egui::TextureId;
use opencv::{
    core::{Mat, MatTraitConst, Point, Point2f, Point3f, Size, Vector},
    imgcodecs, imgproc,
};

use crate::{
    video_pipelines::{
        copy_to_ecs::CopyToEcsPipeline, undistort::UndistortPipeline, AppPipelineExt,
        SerialPipeline,
    },
    video_stream,
};

pub const POI_SIZE: f64 = 50.0;

const CONTOUR_MIN_AREA: f64 = 20.0;
const MIN_CONTOUR_LENGTH: f64 = 35.0;
const MIN_CONTOUR_POINTS: usize = 25;

const MIN_LINE_SEPERATION: f32 = 4.0;
const MAX_LINE_SEPERATION: f32 = 25.0;
const MAX_LINE_ANGLE_DIFFERENCE: f32 = 5.0f32.to_radians();

const PVC_PIPE_WIDTH_METERS: f32 = 0.021336;

pub struct ShipwreckMeasurementPlugin;

impl Plugin for ShipwreckMeasurementPlugin {
    fn build(&self, app: &mut App) {
        let (tx, rx) = crossbeam::channel::bounded(10);

        app.insert_resource(AsyncImageProcessingChannels(tx, rx))
            .register_video_pipeline::<SerialPipeline<(UndistortPipeline, CopyToEcsPipeline<ShipwreckImageOpenCV>)>>("Measure Shipwreck")
            .add_observer(init_shipwreck_entity)
            .add_systems(Update, read_back_results);

        app.world_mut().spawn(ShipwreckImageOpenCV {
            mat: imgcodecs::imread_def("input1.png").unwrap(),
        });
    }
}

#[derive(Component)]
pub struct ShipwreckImageOpenCV {
    mat: Mat,
}

impl From<&Mat> for ShipwreckImageOpenCV {
    fn from(mat: &Mat) -> Self {
        Self { mat: mat.clone() }
    }
}

#[derive(Component, Clone)]
pub struct ShipwreckImage {
    pub image_handle: Handle<Image>,
    pub egui_texture: TextureId,
}

#[derive(Component, Default, Clone)]
pub struct ShipwreckMeasurementPOIs {
    pub reference_point: Option<Vec2>,
    pub measurement_start: Option<Vec2>,
    pub measurement_end: Option<Vec2>,
}

#[derive(Component, Default, Clone)]
pub struct ShipwreckMeasurementResult {
    pub length: Meters,
}

#[derive(Resource)]
struct AsyncImageProcessingChannels(
    Sender<(Entity, ShipwreckMeasurementResult)>,
    Receiver<(Entity, ShipwreckMeasurementResult)>,
);

#[derive(Event, Debug)]
pub struct ComputeShipwreckMeasurement;

fn init_shipwreck_entity(
    trigger: Trigger<OnInsert, ShipwreckImageOpenCV>,
    mut cmds: Commands,
    mut egui_contexts: EguiContexts,
    mut images: ResMut<Assets<Image>>,
    query: Query<&ShipwreckImageOpenCV>,
) {
    let Ok(image_opencv) = query.get(trigger.entity()) else {
        error!("Got bad oninsert for ShipwreckImageOpenCV");
        return;
    };

    let mut image = Image::default();
    let Ok(()) = video_stream::mat_to_image(&image_opencv.mat, &mut image) else {
        error!("error converting mat to image");
        return;
    };

    let image_handle = images.add(image);
    let egui_texture = egui_contexts.add_image(image_handle.clone_weak());

    cmds.entity(trigger.entity())
        .insert((
            ShipwreckImage {
                image_handle,
                egui_texture,
            },
            ShipwreckMeasurementPOIs::default(),
        ))
        .observe(compute_measurements);
}

fn compute_measurements(
    trigger: Trigger<ComputeShipwreckMeasurement>,
    query: Query<(&ShipwreckImageOpenCV, &ShipwreckMeasurementPOIs)>,
    channels: Res<AsyncImageProcessingChannels>,
) {
    let Ok((image, pois)) = query.get(trigger.entity()) else {
        error!("Got bad ComputeShipwreckMeasurement");
        return;
    };

    let entity = trigger.entity();
    let mat = image.mat.clone();
    let pois = pois.clone();
    let tx = channels.0.clone();

    thread::spawn(move || {
        let res = measurement_algo(&mat, pois);

        match res {
            Ok(res) => {
                let _ = tx.send((entity, res));
            }
            Err(err) => error!("Shipwreck measurement failed: {err:?}"),
        }
    });
}

fn read_back_results(mut cmds: Commands, channels: Res<AsyncImageProcessingChannels>) {
    for (entity, measurement) in channels.1.try_iter() {
        cmds.entity(entity).insert(measurement);
    }
}

pub fn measurement_algo(
    mat: &Mat,
    pois: ShipwreckMeasurementPOIs,
) -> anyhow::Result<ShipwreckMeasurementResult> {
    imgcodecs::imwrite_def("input.png", &mat).context("save")?;

    let reference_poi = pois
        .reference_point
        .context("Reference point not specified")?;
    let measurement_start = pois
        .measurement_start
        .context("Measurement start not specified")?;
    let measurement_end = pois
        .measurement_end
        .context("Measurement end not specified")?;

    let roi_small = mat
        .roi(opencv::core::Rect::new(
            (reference_poi.x - POI_SIZE as f32) as i32,
            (reference_poi.y - POI_SIZE as f32) as i32,
            (POI_SIZE * 2.0) as i32,
            (POI_SIZE * 2.0) as i32,
        ))
        .context("Get ROI")?
        // TODO: Figure out how to avoid this
        .clone_pointee();

    let lines = find_lines(&roi_small).context("Find Lines")?;
    let [line_a, line_b] = choose_parallel_lines(&lines).context("Choose parallel lines")?;

    vis_lines(
        &roi_small,
        &Vector::from_slice(&[line_a, line_b]),
        "lines_coarse.png",
    )
    .context("Vis lines")?;

    let measurement_px = measurement_start.distance(measurement_end);
    Ok(ShipwreckMeasurementResult {
        length: Meters(measurement_px / (line_a.x - line_b.x).abs() * PVC_PIPE_WIDTH_METERS),
    })
}

pub fn choose_parallel_lines(lines: &Vector<Point3f>) -> anyhow::Result<[Point3f; 2]> {
    if lines.len() < 2 {
        bail!("Not enough lines found");
    }

    let mut first_line = 0;
    while first_line < lines.len() {
        let line_a = lines.get(first_line).unwrap();
        let Point3f {
            x: radius_a,
            y: theta_a,
            z: votes_a,
        } = line_a;
        info!("Votes Line A: {votes_a}");

        let mut line_b = None;
        for line in lines.iter().skip(first_line + 1) {
            if (line.x - radius_a).abs() > MAX_LINE_SEPERATION {
                continue;
            }
            if (line.x - radius_a).abs() > MIN_LINE_SEPERATION {
                line_b = Some(line);
                break;
            }
        }

        let Some(line_b) = line_b else {
            bail!("Secondary line not found");
        };
        let Point3f {
            x: radius_b,
            y: theta_b,
            z: votes_b,
        } = line_b;

        info!("Votes Line B: {votes_b}");

        if (theta_b - theta_a).abs() > MAX_LINE_ANGLE_DIFFERENCE {
            first_line += 1;
            warn!("Lines are not parallel");
            continue;
        }

        return Ok([line_a, line_b]);
    }
    bail!("No parallel lines were found");
}

// TODO: consider using the probalistic verson of hough lines
pub fn find_lines(mat: &Mat) -> anyhow::Result<Vector<Point3f>> {
    let mut lines = Vector::<Point3f>::default();

    let edges = canny(mat).context("Edges")?;

    imgproc::hough_lines_def(&edges, &mut lines, 1.0, 1.0f64.to_radians(), 50)
        .context("Hough Lines")?;

    println!("Found {} lines", lines.len());

    Ok(lines)
}

pub fn canny(mat: &Mat) -> anyhow::Result<Mat> {
    let mut blur = Mat::default();
    let mut edges = Mat::default();

    imgproc::blur_def(&mat, &mut blur, Size::new(3, 3)).context("Blur")?;
    imgproc::canny_def(&blur, &mut edges, 50.0, 200.0).context("Canny")?;

    imgcodecs::imwrite_def("canny.png", &edges).context("save")?;

    Ok(edges)
}

pub fn vis_lines(mat: &Mat, lines: &Vector<Point3f>, file: &str) -> anyhow::Result<()> {
    let mut vis = mat.clone();

    for line in lines.iter() {
        let radius = line.x;
        let theta = line.y;
        let votes = line.z;

        info!(
            "radius: {:.2}, theta: {:.2}, votes: {votes}",
            radius,
            theta.to_degrees()
        );

        let a = theta.cos();
        let b = theta.sin();

        let x_0 = a * radius;
        let y_0 = b * radius;

        let x_1 = x_0 + 1000.0 * -b;
        let y_1 = y_0 + 1000.0 * a;

        let x_2 = x_0 - 1000.0 * -b;
        let y_2 = y_0 - 1000.0 * a;

        imgproc::line_def(
            &mut vis,
            Point::new(x_1 as i32, y_1 as i32),
            Point::new(x_2 as i32, y_2 as i32),
            (0, 0, 255).into(),
        )
        .context("draw line")?;
    }

    imgcodecs::imwrite_def(file, &vis).context("write img")?;

    Ok(())
}
