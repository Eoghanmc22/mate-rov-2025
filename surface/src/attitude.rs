use bevy::{
    math::vec3,
    prelude::*,
    render::{
        camera::RenderTarget,
        mesh::shape::Cylinder,
        render_resource::{
            Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
        },
        view::RenderLayers,
    },
};
use bevy_egui::EguiContexts;
use common::components::{Motors, Orientation, Robot};
use egui::TextureId;
use motor_math::{ErasedMotorId, Motor, MotorConfig};

pub struct AttitudePlugin;

impl Plugin for AttitudePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup)
            .add_systems(Update, (update_motor_conf, draw_axis, rotator_system))
            .insert_resource(GizmoConfig {
                render_layers: RENDER_LAYERS,
                ..default()
            });
    }
}

#[derive(Resource, Debug, Clone)]
pub struct OrientationDisplay(pub Handle<Image>, pub TextureId);
#[derive(Component)]
struct OrientationDisplayMarker;
#[derive(Component)]
struct MotorMarker(ErasedMotorId);

const RENDER_LAYERS: RenderLayers = RenderLayers::layer(1);

fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut egui_context: EguiContexts,
) {
    let size = Extent3d {
        width: 512,
        height: 512,
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
        PointLightBundle {
            point_light: PointLight {
                intensity: 4000.0,
                shadows_enabled: true,
                ..default()
            },
            transform: Transform::from_xyz(4.0, 4.0, 8.0),
            ..default()
        },
        RENDER_LAYERS,
    ));

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(5.0, -5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Z),
            camera: Camera {
                // render before the "main pass" camera
                order: -1,
                target: RenderTarget::Image(image_handle.clone()),
                ..default()
            },
            ..default()
        },
        RENDER_LAYERS,
    ));

    let texture = egui_context.add_image(image_handle.clone_weak());
    commands.insert_resource(OrientationDisplay(image_handle, texture));
}

fn add_motor_conf(
    motor_conf: &MotorConfig<ErasedMotorId>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,

    render_layer: RenderLayers,
) {
    // TODO/FIXME: This assumes x3d motor conf
    let frt = motor_conf.motor(&0).unwrap();

    commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(
                    shape::Box::new(
                        frt.position.x * 2.0,
                        frt.position.y * 2.0,
                        frt.position.z * 2.0,
                    )
                    .into(),
                ),
                material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
                transform: Transform::from_scale(Vec3::splat(3.5)),
                ..default()
            },
            OrientationDisplayMarker,
            render_layer,
        ))
        .with_children(|builder| {
            for (motor_id, motor) in motor_conf.motors() {
                add_motor(*motor_id, motor, builder, meshes, materials_pbr);
            }
        });
}

fn add_motor(
    motor_id: ErasedMotorId,
    motor: &Motor,

    builder: &mut ChildBuilder,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    builder.spawn((
        PbrBundle {
            mesh: meshes.add(
                Cylinder {
                    radius: 0.005,
                    height: 1.0,
                    ..default()
                }
                .into(),
            ),
            material: materials_pbr.add(Color::GREEN.into()),
            transform: Transform::from_translation(Vec3::from(
                motor.position + motor.orientation / 2.0,
            ))
            .looking_to(motor.orientation.into(), (-motor.position).into())
                * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        MotorMarker(motor_id),
        RENDER_LAYERS,
    ));

    builder.spawn((
        PbrBundle {
            mesh: meshes.add(
                Cylinder {
                    radius: 0.1,
                    height: 0.1,
                    ..default()
                }
                .into(),
            ),
            material: materials_pbr.add(Color::DARK_GRAY.into()),
            transform: Transform::from_translation(Vec3::from(motor.position))
                .looking_to(motor.orientation.into(), (-motor.position).into())
                * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        MotorMarker(motor_id),
        RENDER_LAYERS,
    ));
}

fn update_motor_conf(
    mut commands: Commands,
    motor_conf: Query<&Motors, Changed<Motors>>,
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

fn draw_axis(mut gizmos: Gizmos) {}

fn rotator_system(
    robot: Query<&Orientation, With<Robot>>,
    mut query: Query<&mut Transform, With<OrientationDisplayMarker>>,
    mut gizmos: Gizmos,
) {
    if let Ok(orientation) = robot.get_single() {
        for mut transform in &mut query {
            transform.rotation = orientation.0;
        }

        gizmos.rect(
            Vec3::ZERO,
            orientation.0,
            Vec2::splat(5.0),
            Color::DARK_GRAY,
        );

        for i in 1..=9 {
            let y = i as f32 / 2.0 - 2.5;

            gizmos.line(
                orientation.0 * vec3(-2.5, y, 0.0),
                orientation.0 * vec3(2.5, y, 0.0),
                if y != 0.0 {
                    Color::DARK_GRAY
                } else {
                    Color::RED
                },
            );
        }

        for i in 1..=9 {
            let x = i as f32 / 2.0 - 2.5;

            gizmos.line(
                orientation.0 * vec3(x, -2.5, 0.0),
                orientation.0 * vec3(x, 2.5, 0.0),
                if x != 0.0 {
                    Color::DARK_GRAY
                } else {
                    Color::GREEN
                },
            );
        }

        gizmos.line(
            orientation.0 * vec3(0.0, 0.0, -2.5),
            orientation.0 * vec3(0.0, 0.0, 2.5),
            Color::BLUE,
        );
    }
}
