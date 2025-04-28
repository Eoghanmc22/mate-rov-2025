use bevy::{
    image::TextureAccessError,
    math::Vec3A,
    pbr::wireframe::{WireframeMaterial, WireframePlugin},
    prelude::*,
    render::{
        camera::RenderTarget,
        render_resource::{
            Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
        },
    },
};
use bevy_egui::EguiContexts;
use common::components::{Orientation, Robot};
use egui::TextureId;

use crate::{
    layer_allocator::next_render_layer, video_display_2d_master::VideoMasterMarker,
    video_stream::ImageHandle, DARK_MODE,
};

// TODO: Rotate camera to current robot orientation when taking image
pub struct PhotoSpherePlugin;

impl Plugin for PhotoSpherePlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(spawn_photo_sphere)
            .add_observer(take_photo_sphere_image)
            .add_plugins(WireframePlugin);
    }
}

#[derive(Component, Debug, Clone)]
pub struct PhotoSphere {
    pub view_texture: Handle<Image>,
    pub view_texture_egui: TextureId,

    pub photo_sphere: Handle<Image>,
}

#[derive(Component, Debug, Clone)]
pub struct PhotoSphereCameraMarker;

// Trigger on photosphere entity
#[derive(Event, Debug, Clone)]
pub struct UpdatePhotoSphere {
    pub image: Image,
    pub fov_degrees: f32,
    pub quat: Quat,
}

// Trigger on corosponding robot entity
#[derive(Event, Debug, Clone)]
pub struct SpawnPhotoSphere;

// Trigger on corosponding robot entity
#[derive(Event, Debug, Clone)]
pub struct TakePhotoSphereImage;

// Trigger on photosphere entity
#[derive(Event, Debug, Clone)]
pub struct RotatePhotoSphere(pub Vec2);

fn spawn_photo_sphere(
    event: Trigger<SpawnPhotoSphere>,

    robot: Query<(), With<Robot>>,

    mut cmds: Commands,
    mut images: ResMut<Assets<Image>>,
    mut egui_context: EguiContexts,

    mut ambient_light: ResMut<AmbientLight>,

    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    mut materials_wireframe: ResMut<Assets<WireframeMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Ok(()) = robot.get(event.entity()) else {
        error!("Tried to setup photosphere on non robot entity");
        return;
    };

    let layer = next_render_layer();

    let view_size = Extent3d {
        // FIXME: why is this using such a weird size?
        width: 920,
        height: 920,
        ..default()
    };

    let photo_sphere_size = Extent3d {
        width: 4096,
        height: 4096,
        ..default()
    };

    // This is the texture that will be rendered to.
    let mut view_image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size: view_size,
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_DST
                | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };

    // fill image.data with zeroes
    view_image.resize(view_size);

    let view_image_handle = images.add(view_image);

    // This is the texture that will be rendered to.
    let mut photo_sphere_image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size: photo_sphere_size,
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_DST
                | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };

    // fill image.data with zeroes
    photo_sphere_image.resize(photo_sphere_size);

    let photo_sphere_image_handle = images.add(photo_sphere_image);

    let view_image_texture = egui_context.add_image(view_image_handle.clone_weak());
    let photo_sphere = cmds
        .spawn(PhotoSphere {
            view_texture: view_image_handle.clone(),
            view_texture_egui: view_image_texture,
            photo_sphere: photo_sphere_image_handle.clone(),
        })
        .observe(update_photo_sphere)
        .observe(rotate_camera)
        .with_children(|cmds| {
            cmds.spawn((
                Camera3d::default(),
                Camera {
                    // render before the "main pass" camera
                    order: -1,
                    target: RenderTarget::Image(view_image_handle),
                    ..default()
                },
                layer.clone(),
                PhotoSphereCameraMarker,
            ));

            cmds.spawn((
                PointLight {
                    shadows_enabled: true,
                    intensity: if DARK_MODE { 1_000_000.0 } else { 4_000_000.0 },
                    ..default()
                },
                layer.clone(),
            ));
            if !DARK_MODE {
                ambient_light.brightness *= 7.0;
            }

            cmds.spawn((
                Mesh3d(meshes.add(Sphere::new(1.0).mesh().ico(5).unwrap())),
                MeshMaterial3d(materials_pbr.add(photo_sphere_image_handle)),
                layer.clone(),
            ));

            cmds.spawn((
                Mesh3d(meshes.add(Sphere::new(1.1).mesh().ico(5).unwrap())),
                MeshMaterial3d(materials_wireframe.add(WireframeMaterial::default())),
                layer.clone(),
            ));
        })
        .id();
    cmds.entity(event.entity()).add_child(photo_sphere);
}

fn update_photo_sphere(
    event: Trigger<UpdatePhotoSphere>,
    mut cmds: Commands,
    query: Query<&PhotoSphere>,
    cameras: Query<Entity, With<PhotoSphereCameraMarker>>,
    mut images: ResMut<Assets<Image>>,
) {
    let Ok(photosphere) = query.get(event.entity()) else {
        return;
    };

    let Some(photosphere) = images.get_mut(&photosphere.photo_sphere) else {
        return;
    };

    let res = update_photo_sphere_inner(photosphere, event.event());

    match res {
        Ok(()) => {}
        Err(err) => panic!("Error in update_photo_sphere_inner: {err:?}"),
    }

    // TODO: relate the camera to the photosphere entity
    for camera in cameras.iter() {
        cmds.entity(camera)
            .insert(Transform::from_rotation(event.event().quat));
    }
}

fn rotate_camera(
    event: Trigger<RotatePhotoSphere>,
    photosphere: Query<&Children, With<PhotoSphere>>,
    mut camera: Query<&mut Transform, With<PhotoSphereCameraMarker>>,
) {
    let Ok(children) = photosphere.get(event.entity()) else {
        error!("get children of photosphere");
        return;
    };

    for child in children {
        let Ok(mut transform) = camera.get_mut(*child) else {
            continue;
        };

        let Vec2 { x, y } = event.event().0;
        transform.rotate_z(x);
        transform.rotate_local_x(y);
    }
}

fn take_photo_sphere_image(
    event: Trigger<TakePhotoSphereImage>,
    mut cmds: Commands,
    robot: Query<(&Orientation, Option<&Children>), With<Robot>>,
    master_camera: Query<&ImageHandle, With<VideoMasterMarker>>,
    photo_spheres: Query<Entity, With<PhotoSphere>>,
    images: Res<Assets<Image>>,
) {
    let Ok((orientation, children)) = robot.get(event.entity()) else {
        error!("Get robot orientation for image");
        return;
    };

    let mut is_taken = false;

    for child in children.iter().flat_map(|it| it.into_iter()) {
        let Ok(photosphere) = photo_spheres.get(*child) else {
            continue;
        };

        let Ok(image_handle) = master_camera.get_single() else {
            error!("Get image from master camera");
            return;
        };

        let Some(image) = images.get(&image_handle.0) else {
            error!("Get image from image handle");
            return;
        };

        cmds.entity(photosphere).trigger(UpdatePhotoSphere {
            image: image.clone(),
            fov_degrees: 100.0,
            quat: orientation.0,
        });

        is_taken = true;
    }

    if !is_taken {
        cmds.entity(event.entity())
            .trigger(SpawnPhotoSphere)
            .trigger(TakePhotoSphereImage);
    }
}

/// Update `photo_sphere` (an equirectangular pano) by “painting in” the newly captured
/// `event.image` with known `fov_degrees` and camera `quat`.
pub fn update_photo_sphere_inner(
    photo_sphere: &mut Image,
    event: &UpdatePhotoSphere,
) -> Result<(), TextureAccessError> {
    let src = &event.image;
    let (w_src, h_src) = (src.width() as f32, src.height() as f32);
    let aspect = w_src / h_src;

    let w_dst = photo_sphere.width();
    let h_dst = photo_sphere.height();

    // Convert fov to radians once, and half‐angles
    let fov_y = event.fov_degrees.to_radians();
    let tan_half_fov_y = (fov_y * 0.5).tan();
    let tan_half_fov_x = tan_half_fov_y * aspect;

    // Precompute inverse quaternion
    let inv_cam = event.quat.inverse();

    for y in 0..h_dst {
        // Normalized [0,1]
        let v = (y as f32 + 0.5) / (h_dst as f32);

        // inclination measured from +Y down toward −Y
        // v=0 → top (north pole), v=1 → bottom (south pole)
        let incl = v * core::f32::consts::PI;
        let sin_incl = incl.sin();
        let cos_incl = incl.cos();

        for x in 0..w_dst {
            let u = (x as f32 + 0.5) / (w_dst as f32);

            // azimuth around Y axis: u=0 → −X, u=0.5 → +X, u=1→−X wrap
            let az = (0.5 - u) * core::f32::consts::TAU;
            let sin_az = az.sin();
            let cos_az = az.cos();

            // point on unit sphere in world‐space
            let world_dir = Vec3A::new(
                cos_az * sin_incl, // x
                cos_incl,          // y
                sin_az * sin_incl, // z
            );

            // rotate into camera‐local space
            let cam_dir = inv_cam * world_dir;

            // Cull rays behind the camera
            if cam_dir.z <= 0.0 {
                continue;
            }

            // project onto the image plane: (x/z, y/z)
            let x_ndc = cam_dir.x / cam_dir.z;
            let y_ndc = cam_dir.y / cam_dir.z;

            // check against frustum
            if x_ndc.abs() > tan_half_fov_x || y_ndc.abs() > tan_half_fov_y {
                continue;
            }

            // Convert to normalized [0,1] source‐image UVs
            // x_ndc ∈ [−tanX, +tanX] → u_src ∈ [0,1]
            let u_src = (x_ndc / (2.0 * tan_half_fov_x)) + 0.5;
            // y_ndc ∈ [−tanY, +tanY] → v_src ∈ [0,1], note we flip Y
            let v_src = 0.5 - (y_ndc / (2.0 * tan_half_fov_y));

            // Map to pixel‐center coordinates in the source
            let fx = u_src * (w_src - 1.0);
            let fy = v_src * (h_src - 1.0);

            // Bilinear sample
            let x0 = fx.floor().clamp(0.0, w_src - 1.0);
            let y0 = fy.floor().clamp(0.0, h_src - 1.0);
            let x1 = (x0 + 1.0).min(w_src - 1.0);
            let y1 = (y0 + 1.0).min(h_src - 1.0);

            let dx = fx - x0;
            let dy = fy - y0;

            let c00 = src.get_color_at(x0 as u32, y0 as u32)?.to_linear();
            let c10 = src.get_color_at(x1 as u32, y0 as u32)?.to_linear();
            let c01 = src.get_color_at(x0 as u32, y1 as u32)?.to_linear();
            let c11 = src.get_color_at(x1 as u32, y1 as u32)?.to_linear();

            // lerp in x, then y
            let lerp_x0 = LinearRgba::mix(&c00, &c10, dx);
            let lerp_x1 = LinearRgba::mix(&c01, &c11, dx);
            let final_color = LinearRgba::mix(&lerp_x0, &lerp_x1, dy);

            photo_sphere.set_color_at(x, y, final_color.into())?;
        }
    }

    Ok(())
}
