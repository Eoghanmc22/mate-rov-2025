use bevy::{
    pbr::wireframe::{Wireframe, WireframeColor},
    prelude::*,
    render::{
        camera::RenderTarget,
        render_resource::{
            Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
        },
        view::RenderLayers,
    },
};
use bevy_egui::EguiContexts;
use common::components::{Orientation, Robot, RobotId};
use egui::TextureId;

use crate::{
    layer_allocator::next_render_layer, video_display_2d_master::VideoMasterMarker,
    video_stream::ImageHandle,
};

// TODO: Consider switching to rendering each image to a plane instead of projecting to a sphere
pub struct PhotoSpherePlugin;

impl Plugin for PhotoSpherePlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(spawn_photo_sphere)
            .add_observer(take_photo_sphere_image);
    }
}

#[derive(Component, Debug, Clone)]
pub struct PhotoSphere {
    pub view_texture: Handle<Image>,
    pub view_texture_egui: TextureId,

    pub images: Vec<(Handle<Image>, TextureId)>,
    pub materials: Vec<Handle<StandardMaterial>>,
    pub square_mesh: Handle<Mesh>,
}

#[derive(Component, Debug, Clone)]
pub struct PhotoSphereCameraMarker;

// Trigger on photosphere entity
#[derive(Event, Debug, Clone)]
pub struct UpdatePhotoSphere {
    pub image: Image,
    // Radians
    pub fov: f32,
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

    robot: Query<&RobotId, With<Robot>>,
    mut cmds: Commands,
    mut images: ResMut<Assets<Image>>,
    mut egui_context: EguiContexts,

    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Ok(robot_id) = robot.get(event.entity()) else {
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
    let view_image_texture = egui_context.add_image(view_image_handle.clone_weak());

    cmds.spawn((
        Name::new("Photosphere"),
        Transform::default(),
        Visibility::default(),
        PhotoSphere {
            view_texture: view_image_handle.clone(),
            view_texture_egui: view_image_texture,
            materials: vec![],
            images: vec![],
            square_mesh: meshes.add(Plane3d::new(Vec3::Z, Vec2::splat(1.0))),
        },
        layer.clone(),
        *robot_id,
    ))
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
            Projection::Perspective(PerspectiveProjection {
                fov: 120.0f32.to_radians(),
                ..default()
            }),
            Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            layer.clone(),
            PhotoSphereCameraMarker,
        ));

        cmds.spawn((
            Mesh3d(meshes.add(Sphere::new(-5.0).mesh().uv(32, 18))),
            Wireframe,
            WireframeColor {
                color: Color::BLACK,
            },
            layer.clone(),
        ));
    });
}

fn update_photo_sphere(
    event: Trigger<UpdatePhotoSphere>,
    mut cmds: Commands,
    mut query: Query<(Entity, &mut PhotoSphere, &Children, &RenderLayers)>,
    cameras: Query<Entity, With<PhotoSphereCameraMarker>>,

    mut images: ResMut<Assets<Image>>,
    mut egui_context: EguiContexts,

    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Ok((entity, mut photosphere, children, layer)) = query.get_mut(event.entity()) else {
        return;
    };

    let update = event.event().clone();
    let mut size = update.image.size_f32().normalize();
    // let mut size = update.image.size_f32();
    // size /= size.y;

    let radius = 1.0;
    size *= (update.fov / 2.0).tan() * radius;

    let image_handle = images.add(update.image);
    let texture = egui_context.add_image(image_handle.clone_weak());

    cmds.entity(entity).with_child((
        Mesh3d(photosphere.square_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(image_handle.clone()),
            unlit: true,
            ..default()
        })),
        Transform {
            translation: update.quat * Vec3::NEG_Z * radius,
            rotation: update.quat,
            scale: size.extend(1.0),
        },
        layer.clone(),
    ));

    photosphere.images.push((image_handle, texture));

    for child in children {
        if let Ok(camera) = cameras.get(*child) {
            cmds.entity(camera)
                .insert(Transform::default().looking_at(update.quat * Vec3::NEG_Z, Vec3::Z));
        }
    }
}

fn rotate_camera(
    event: Trigger<RotatePhotoSphere>,
    photosphere: Query<&Children, With<PhotoSphere>>,
    mut camera: Query<(&mut Transform, &Projection), With<PhotoSphereCameraMarker>>,
) {
    let Ok(children) = photosphere.get(event.entity()) else {
        error!("get children of photosphere");
        return;
    };

    let mut did_rotate = false;

    for child in children {
        let Ok((mut transform, Projection::Perspective(proj))) = camera.get_mut(*child) else {
            continue;
        };

        did_rotate = true;

        info!("Rotate_camera by: {:?}", event.event().0);

        let Vec2 { x, y } = event.event().0 * proj.fov;
        transform.rotate_z(x);
        transform.rotate_local_x(y);
    }

    if !did_rotate {
        error!("Did not rotate");
    }
}

fn take_photo_sphere_image(
    event: Trigger<TakePhotoSphereImage>,
    mut cmds: Commands,
    robot: Query<(&Orientation, &RobotId), With<Robot>>,
    master_camera: Query<&ImageHandle, With<VideoMasterMarker>>,
    photo_spheres: Query<(Entity, &RobotId), With<PhotoSphere>>,
    images: Res<Assets<Image>>,
) {
    let Ok((orientation, robot_id)) = robot.get(event.entity()) else {
        error!("Get robot orientation for image");
        return;
    };

    let mut is_taken = false;

    for (photosphere, other_robot_id) in photo_spheres.iter() {
        if robot_id != other_robot_id {
            continue;
        }

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
            fov: 100.0f32.to_radians(),
            quat: Quat::from_rotation_x(90f32.to_radians()) * orientation.0,
        });

        is_taken = true;
    }

    if !is_taken {
        cmds.entity(event.entity())
            .trigger(SpawnPhotoSphere)
            .trigger(TakePhotoSphereImage);
    }
}
