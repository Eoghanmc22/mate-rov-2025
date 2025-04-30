use std::thread;

use anyhow::Context;
use bevy::prelude::*;
use bevy_egui::EguiContexts;
use common::types::units::Meters;
use crossbeam::channel::{Receiver, Sender};
use egui::TextureId;
use opencv::{
    core::{Mat, Point2f, Vector},
    imgcodecs, imgproc,
};

use crate::{
    video_pipelines::{
        copy_to_ecs::CopyToEcsPipeline, undistort::UndistortPipeline, AppPipelineExt,
        SerialPipeline,
    },
    video_stream,
};

const POI_SIZE: f64 = 100.0;
const CONTOUR_MIN_AREA: f64 = 20.0;
const MIN_CONTOUR_LENGTH: f64 = 50.0;
const MIN_CONTOUR_POINTS: usize = 20;
const PVC_PIPE_WIDTH_METERS: f32 = 0.021336;

pub struct ShipwreckMeasurementPlugin;

impl Plugin for ShipwreckMeasurementPlugin {
    fn build(&self, app: &mut App) {
        app.register_video_pipeline::<SerialPipeline<(UndistortPipeline, CopyToEcsPipeline<ShipwreckImageOpenCV>)>>("Measure Shipwreck")
        .add_observer(init_shipwreck_entity)
        .add_systems(Update, read_back_results);
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

fn measurement_algo(
    mat: &Mat,
    pois: ShipwreckMeasurementPOIs,
) -> anyhow::Result<ShipwreckMeasurementResult> {
    let reference_poi = convert_poi(
        pois.reference_point
            .context("Reference point not specified")?,
    );
    let measurement_start = pois
        .measurement_start
        .context("Measurement start not specified")?;
    let measurement_end = pois
        .measurement_end
        .context("Measurement end not specified")?;

    let mut edges = Mat::default();
    let mut contours = Vector::<Vector<Point2f>>::default();

    imgproc::canny_def(&mat, &mut edges, 100.0, 100.0).context("Canny")?;
    imgcodecs::imwrite_def("canny.png", &edges).context("save")?;

    imgproc::find_contours_def(
        &edges,
        &mut contours,
        imgproc::RETR_LIST,
        // TODO: Are the other approximation modes better
        imgproc::CHAIN_APPROX_SIMPLE,
    )
    .context("Find contours")?;

    info!("Found {} contours", contours.len());

    let mut contour_img = mat.clone();
    imgproc::draw_contours_def(&mut contour_img, &contours, -1, (0, 0, 255).into())
        .context("Draw Contours")?;
    imgcodecs::imwrite_def("all_contours.png", &edges).context("save")?;

    let mut good_contours: Vector<Vector<Point2f>> = Vector::new();
    let mut arc_lengths = vec![];

    for contour in contours {
        let moments = imgproc::moments_def(&contour).context("Get moments")?;
        let area = moments.m00;

        // Contour too small
        if area < CONTOUR_MIN_AREA {
            continue;
        }

        // TODO: Might be hard to get a point in the region
        let dist =
            imgproc::point_polygon_test(&contour, reference_poi, true).context("Point test")?;

        if dist.abs() > POI_SIZE {
            continue;
        }

        let mut good_points = Vector::<Point2f>::default();

        for point in contour {
            if (point.x - reference_poi.x)
                .abs()
                .max((point.y - reference_poi.y).abs())
                < POI_SIZE as f32
            {
                good_points.push(point);
            }
        }

        if good_points.len() < MIN_CONTOUR_POINTS {
            continue;
        }

        let arc_length = imgproc::arc_length(&good_points, false).context("Arc Length")?;

        if arc_length < MIN_CONTOUR_LENGTH {
            continue;
        }

        arc_lengths.push((arc_length, good_contours.len()));
        good_contours.push(good_points);
    }

    let mut contour_img = mat.clone();
    imgproc::draw_contours_def(&mut contour_img, &good_contours, -1, (0, 0, 255).into())
        .context("Draw Contours")?;
    imgcodecs::imwrite_def("good_contours.png", &edges).context("save")?;

    arc_lengths.sort_by(|(a, _), (b, _)| f64::total_cmp(a, b).reverse());

    info!("Found {} good contours", good_contours.len());

    let (best_two, _) = arc_lengths
        .split_first_chunk::<2>()
        .context("not enough good contours")?;

    let a = &good_contours
        .get(best_two[0].1)
        .context("Get good contour")?;
    let b = &good_contours
        .get(best_two[1].1)
        .context("Get good contour")?;

    {
        let mut best_countours = Vector::<Vector<Point2f>>::default();
        best_countours.push(a.clone());
        best_countours.push(b.clone());

        let mut contour_img = mat.clone();
        imgproc::draw_contours_def(&mut contour_img, &best_countours, -1, (0, 0, 255).into())
            .context("Draw Contours")?;
        imgcodecs::imwrite_def("top_contours.png", &edges).context("save")?;
    }

    let mut average_distance = 0.0;

    for point in b {
        let dist =
            imgproc::point_polygon_test(a, point, true).context("Point test good contours")?;
        average_distance += dist.abs();
    }

    average_distance /= b.len() as f64;

    let measurement_px = measurement_start.distance(measurement_end);

    Ok(ShipwreckMeasurementResult {
        length: Meters(measurement_px / average_distance as f32 * PVC_PIPE_WIDTH_METERS),
    })
}

fn convert_poi(point: Vec2) -> Point2f {
    Point2f::new(point.x, point.y)
}
