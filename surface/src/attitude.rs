use bevy::{
    color::palettes::css,
    math::{vec3, Vec3A},
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
use common::components::{Orientation, OrientationTarget, Robot, Thrusters};
use egui::TextureId;
use motor_math::{glam::MotorGlam, x3d::X3dMotorId, Direction, ErasedMotorId, Motor, MotorConfig};

use crate::DARK_MODE;

const RENDER_LAYERS: RenderLayers = RenderLayers::layer(1);

pub struct AttitudePlugin;

impl Plugin for AttitudePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup)
            .add_systems(Update, (update_motor_conf, rotator_system))
            .insert_gizmo_config(
                AttitudeGizmo,
                GizmoConfig {
                    render_layers: RENDER_LAYERS,
                    ..default()
                },
            );
    }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct AttitudeGizmo;

#[derive(Resource, Debug, Clone)]
pub struct OrientationDisplay(pub Handle<Image>, pub TextureId);
#[derive(Component)]
struct OrientationDisplayMarker;
#[derive(Component)]
struct MotorMarker(ErasedMotorId);

fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut egui_context: EguiContexts,

    mut ambient_light: ResMut<AmbientLight>,

    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let size = Extent3d {
        // width: 512,
        // height: 512,
        // width: 1024,
        // height: 1024,
        // FIXME: why is this using such a weird size?
        width: 920,
        height: 920,
        ..default()
    };

    // This is the texture that will be rendered to.
    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size,
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
    image.resize(size);

    let image_handle = images.add(image);

    // light
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            intensity: if DARK_MODE { 1_000_000.0 } else { 4_000_000.0 },
            ..default()
        },
        Transform::from_xyz(4.0, 4.0, 8.0),
        RENDER_LAYERS,
    ));
    if !DARK_MODE {
        ambient_light.brightness *= 7.0;
    }

    // camera
    commands.spawn((
        Camera3d::default(),
        Camera {
            // render before the "main pass" camera
            order: -1,
            target: RenderTarget::Image(image_handle.clone()),
            ..default()
        },
        Transform::from_xyz(5.0, -5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Z),
        RENDER_LAYERS,
    ));

    // Makes bevy allocate the gpu resources needed, preveinting a >300ms freeze
    // on first connection to robot
    add_motor_conf(
        &MotorConfig::<X3dMotorId, f32>::new(
            MotorGlam {
                position: Vec3A::default(),
                orientation: Vec3A::default(),
                direction: Direction::Clockwise,
            }
            .into(),
            Default::default(),
        )
        .erase(),
        &mut commands,
        &mut meshes,
        &mut materials,
        RENDER_LAYERS,
    );

    let texture = egui_context.add_image(image_handle.clone_weak());
    commands.insert_resource(OrientationDisplay(image_handle, texture));
}

fn add_motor_conf(
    motor_conf: &MotorConfig<ErasedMotorId, f32>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,

    render_layer: RenderLayers,
) {
    // FIXME(low): This assumes x3d motor conf
    let frt = motor_conf.motor(&0).unwrap();

    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                frt.position.x * 2.0 * 1.5,
                frt.position.y * 2.0 * 1.5,
                frt.position.z * 2.0 * 1.5,
            ))),
            MeshMaterial3d(materials_pbr.add(Color::srgb(0.8, 0.7, 0.6))),
            Transform::from_scale(Vec3::splat(3.5)),
            OrientationDisplayMarker,
            render_layer,
        ))
        .with_children(|builder| {
            for (motor_id, motor) in motor_conf.motors() {
                add_motor(*motor_id, &motor.into(), builder, meshes, materials_pbr);
            }
        });
}

fn add_motor(
    motor_id: ErasedMotorId,
    motor: &MotorGlam,

    builder: &mut ChildBuilder,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    builder.spawn((
        Mesh3d(meshes.add(Cylinder {
            radius: 0.005,
            half_height: 0.5,
        })),
        MeshMaterial3d(materials_pbr.add(Color::from(css::GREEN))),
        Transform::from_translation(Vec3::from(motor.position * 1.5 + motor.orientation / 2.0))
            .looking_to(Vec3::from(motor.orientation), Vec3::from(-motor.position))
            * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
        MotorMarker(motor_id),
        RENDER_LAYERS,
    ));

    builder.spawn((
        Mesh3d(meshes.add(Cylinder {
            radius: 0.125,
            half_height: 0.0625,
        })),
        MeshMaterial3d(materials_pbr.add(Color::from(css::DARK_GRAY))),
        Transform::from_translation(Vec3::from(motor.position * 1.5))
            .looking_to(Vec3::from(motor.orientation), Vec3::from(-motor.position))
            * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
        MotorMarker(motor_id),
        RENDER_LAYERS,
    ));
}

fn update_motor_conf(
    mut commands: Commands,
    motor_conf: Query<&Thrusters, Changed<Thrusters>>,
    motors_query: Query<Entity, With<OrientationDisplayMarker>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    for motor_conf in &motor_conf {
        for motor in &motors_query {
            commands.entity(motor).despawn_recursive();
        }

        add_motor_conf(
            &motor_conf.0,
            &mut commands,
            &mut meshes,
            &mut materials,
            RENDER_LAYERS,
        );
    }
}

fn rotator_system(
    robot: Query<(&Orientation, Option<&OrientationTarget>), With<Robot>>,
    mut query: Query<&mut Transform, With<OrientationDisplayMarker>>,
    mut gizmos: Gizmos<AttitudeGizmo>,
) {
    if let Ok((orientation, target)) = robot.get_single() {
        for mut transform in &mut query {
            transform.rotation = orientation.0;
        }

        gizmos.rect(orientation.0, Vec2::splat(5.0), Color::from(css::DARK_GRAY));

        for i in 1..=9 {
            let y = i as f32 / 2.0 - 2.5;

            gizmos.line(
                orientation.0 * vec3(-2.5, y, 0.0),
                orientation.0 * vec3(2.5, y, 0.0),
                if y != 0.0 {
                    Color::from(css::DARK_GRAY)
                } else {
                    Color::from(css::RED)
                },
            );
        }

        for i in 1..=9 {
            let x = i as f32 / 2.0 - 2.5;

            gizmos.line(
                orientation.0 * vec3(x, -2.5, 0.0),
                orientation.0 * vec3(x, 2.5, 0.0),
                if x != 0.0 {
                    Color::from(css::DARK_GRAY)
                } else {
                    Color::from(css::GREEN)
                },
            );
        }

        gizmos.line(
            orientation.0 * vec3(0.0, 0.0, -2.5),
            orientation.0 * vec3(0.0, 0.0, 2.5),
            Color::from(css::BLUE),
        );

        if let Some(&OrientationTarget(up)) = target {
            // gizmos.line(vec3(0.0, 0.0, -5.0), vec3(0.0, 0.0, 5.0), Color::BLUE);
            //
            // gizmos.line(
            //     vec3(0.0, 0.0, 0.0),
            //     orientation.0 * (Vec3::from(up) * 5.0),
            //     Color::YELLOW,
            // );

            let axis_rotation_x = Quat::from_rotation_arc(Vec3::Z, Vec3::X);
            let axis_rotation_y = Quat::from_rotation_arc(Vec3::Z, Vec3::Y);
            let axis_rotation_z = Quat::from_rotation_arc(Vec3::Z, Vec3::Z);
            gizmos.circle(up * axis_rotation_x, 2.0, Color::from(css::RED));
            gizmos.circle(up * axis_rotation_y, 2.0, Color::from(css::GREEN));
            gizmos.circle(up * axis_rotation_z, 2.0, Color::from(css::BLUE));
        }
    }
}
