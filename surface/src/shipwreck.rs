use core::f32;

use anyhow::Context;
use bevy::{math::Mat3A, prelude::*};
use bevy_egui::{EguiContexts, EguiUserTextures};
use egui::{Color32, Id, TextureId};
use egui_plot::{Plot, PlotImage, PlotPoints, Points};

use crate::video_pipelines::{
    copy_to_ecs::{CopyToEcsPipeline, CopyToEcsState},
    save::SavePipeline,
    undistort::{CroppedCameraMatrix, UndistortPipeline},
    AppPipelineExt, SerialPipeline,
};

const POINT_COUNT: usize = 4;
const WIDTH_METERS: f32 = 0.47;
const BOW_LENGTH: f32 = 0.30 * f32::consts::FRAC_1_SQRT_2;

pub struct ShipwreckMeasurementPlugin;

impl Plugin for ShipwreckMeasurementPlugin {
    fn build(&self, app: &mut App) {
        app.register_video_pipeline::<SerialPipeline<(
            UndistortPipeline,
            SavePipeline,
            CopyToEcsPipeline<ShipwreckBundle>,
        )>>("Measure Shipwreck")
            .add_systems(Update, shipwreck_ui);

        // app.world_mut().spawn(ShipwreckImageOpenCV {
        //     mat: imgcodecs::imread_def("input1.png").unwrap(),
        // });
    }
}

#[derive(Bundle)]
pub struct ShipwreckBundle {
    pub image: ShipwreckImage,
    pub pois: ShipwreckMeasurementPOIs,
    pub camera_mat: CroppedCameraMatrix,
}

#[derive(Component, Clone)]
pub struct ShipwreckImage {
    pub image_handle: Handle<Image>,
    pub egui_texture: TextureId,
}

impl<'a> TryFrom<CopyToEcsState<'a>> for ShipwreckBundle {
    type Error = anyhow::Error;

    fn try_from(state: CopyToEcsState<'a>) -> anyhow::Result<Self> {
        let mut image_assets = state
            .world
            .get_resource_mut::<Assets<Image>>()
            .context("Get image asset manager")?;
        let image_handle = image_assets.add(state.img);

        let mut egui_textures = state
            .world
            .get_resource_mut::<EguiUserTextures>()
            .context("Get egui texture manager")?;
        let egui_texture = egui_textures.add_image(image_handle.clone_weak());

        let camera_mat = *state
            .world
            .get::<CroppedCameraMatrix>(state.camera_entity)
            .context("Get camera matrix")?;

        Ok(Self {
            image: ShipwreckImage {
                image_handle,
                egui_texture,
            },
            pois: ShipwreckMeasurementPOIs::default(),
            camera_mat,
        })
    }
}

#[derive(Component, Default, Clone)]
pub struct ShipwreckMeasurementPOIs {
    points: Vec<Vec2>,
}

fn shipwreck_ui(
    mut cmds: Commands,
    mut contexts: EguiContexts,
    mut shiprecks: Query<(
        Entity,
        &ShipwreckImage,
        &mut ShipwreckMeasurementPOIs,
        &CroppedCameraMatrix,
    )>,
    images: Res<Assets<Image>>,
) {
    for (entity, image, mut pois, camera_mat) in shiprecks.iter_mut() {
        let mut open = true;

        let context = contexts.ctx_mut();
        egui::Window::new("Shipwreck")
            .id(Id::new(entity))
            .constrain_to(context.available_rect().shrink(20.0))
            .default_size((230.0, 230.0))
            .open(&mut open)
            .show(context, |ui| {
                ui.label("Corner Order: bottom-left, bottom-right, top-right, top-left");
                ui.label("Known side is bottom/top");

                let response = Plot::new("Shipwreck Plot")
                    .data_aspect(1.0)
                    .min_size(egui::Vec2::new(100.0, 100.0))
                    .width(ui.available_width())
                    .height(ui.available_width())
                    .show(ui, |ui| {
                        let image_size = images
                            .get(&image.image_handle)
                            .map(|it| it.size_f32())
                            .unwrap_or_default();

                        ui.image(PlotImage::new(
                            "Shipwreck",
                            image.egui_texture,
                            [image_size.x as f64 / 2.0, -image_size.y as f64 / 2.0].into(),
                            [image_size.x, image_size.y],
                        ));

                        for (idx, point) in pois.points.iter().enumerate() {
                            ui.points(
                                Points::new(
                                    format!("Point {idx}"),
                                    [point.x as f64, -point.y as f64],
                                )
                                .color(Color32::RED)
                                .radius(3.0)
                                .id(Id::new(idx)),
                            );
                        }

                        ui.polygon(
                            egui_plot::Polygon::new(
                                "Reference Point ROI",
                                pois.points
                                    .iter()
                                    .map(|it| [it.x as f64, -it.y as f64])
                                    .collect::<PlotPoints>(),
                            )
                            .stroke((2.0, Color32::RED)),
                        );
                    });

                if let Some(pointer) = response.response.hover_pos() {
                    if response.response.clicked() {
                        let point = response.transform.value_from_position(pointer);
                        let point = Vec2::new(point.x as f32, -point.y as f32);

                        if pois.points.len() < POINT_COUNT {
                            pois.points.push(point);
                        } else {
                            let closest = pois
                                .points
                                .iter_mut()
                                .min_by(|a, b| {
                                    f32::total_cmp(
                                        &a.distance_squared(point),
                                        &b.distance_squared(point),
                                    )
                                })
                                .unwrap();

                            *closest = point;
                        }
                    }
                }

                if pois.points.len() == POINT_COUNT {
                    let length =
                        measure_length_calibrated(&pois.points, WIDTH_METERS, camera_mat.mat)
                            .unwrap_or(f32::NEG_INFINITY)
                            + BOW_LENGTH;
                    ui.label(format!("Shipwreck Length: {length:.2}m"));
                }
            });

        if !open {
            cmds.entity(entity).despawn_recursive();
        }
    }
}

/// corners: bottom-left, bottom-right, top-right, top-left
/// width: the known real length of the bottom edge (in whatever units you like)
/// returns the real length of the right edge
pub fn measure_length_calibrated(corners: &[Vec2], width: f32, camera_mat: Mat3A) -> Option<f32> {
    dbg!(&camera_mat);

    // build homogeneous points p[i] and rays r[i]=p[i]
    let mut p = [Vec3::ZERO; 4];
    for i in 0..4 {
        p[i] = normalize_point(corners[i], camera_mat).extend(1.0)
    }
    let r = p; // back-projected rays coincide with p

    // vanishing in width direction = intersection of lines (p0,p1) and (p3,p2)
    let l01 = p[0].cross(p[1]);
    let l32 = p[3].cross(p[2]);
    let v_w = l01.cross(l32);

    // vanishing in length direction = intersection of lines (p1,p2) and (p0,p3)
    let l12 = p[1].cross(p[2]);
    let l03 = p[0].cross(p[3]);
    let v_l = l12.cross(l03);

    // plane normal (in camera space)
    let n = v_w.cross(v_l).normalize();

    // αᵢ = 1 / (n·rᵢ)
    let alpha: Vec<f32> = r.iter().map(|ri| 1.0 / n.dot(*ri)).collect();

    // length in camera‐space of the known edge (p0→p1)
    let denom = (r[1] * alpha[1] - r[0] * alpha[0]).length();
    if denom.abs() < 1e-6 {
        return None; // degenerate
    }

    // global scale to make that edge == width
    let scale = width / denom;

    // camera‐space length of the opposite edge (p1→p2)
    let numer = (r[2] * alpha[2] - r[1] * alpha[1]).length();

    // real‐world length
    Some(scale * numer)
}

fn normalize_point(point: Vec2, camera_mat: Mat3A) -> Vec2 {
    Vec2::new(
        (point.x - camera_mat.x_axis.z) / camera_mat.x_axis.x,
        (point.y - camera_mat.y_axis.z) / camera_mat.y_axis.y,
    )
}
