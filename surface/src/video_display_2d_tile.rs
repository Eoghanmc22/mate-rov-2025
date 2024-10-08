use std::mem;

use bevy::{
    color::palettes::css,
    prelude::*,
    render::{camera::Camera as BevyCamera, view::RenderLayers},
};
use common::components::Camera;

const RENDER_LAYERS: RenderLayers = RenderLayers::layer(2);

pub struct VideoDisplay2DPlugin;

impl Plugin for VideoDisplay2DPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VideoDisplay2DSettings>()
            // .init_resource::<VideoTree>()
            .add_systems(Startup, setup)
            .add_systems(
                Update,
                (
                    create_display,
                    update_aspect_ratio.after(create_display),
                    enable_camera,
                ),
            );
    }
}

#[derive(Default, Component)]
struct VideoTree(VideoNode);
enum VideoNode {
    Branch(Vec<VideoNode>),
    Leaf(Entity),
}
#[derive(Default, Clone, Copy)]
enum VideoLayout {
    #[default]
    Horizontal,
    Vertical,
}

impl VideoNode {
    const MAX_CHILDREN: u32 = 3;

    fn insert(&mut self, entity: Entity) {
        let total_children = self.count_children();

        match self {
            VideoNode::Branch(children) => {
                if children.is_empty() {
                    *self = VideoNode::Leaf(entity);
                } else if total_children < Self::MAX_CHILDREN {
                    children.push(VideoNode::Leaf(entity));
                } else {
                    let (_, child) = children
                        .iter_mut()
                        .map(|it| (it.count_children(), it))
                        .min_by_key(|(children, _)| *children)
                        .expect("Branch did not have children");
                    child.insert(entity);
                }
            }
            VideoNode::Leaf(this) => {
                *self = VideoNode::Branch(vec![VideoNode::Leaf(*this), VideoNode::Leaf(entity)]);
            }
        }
    }

    fn remove(&mut self, entity: Entity) {
        match self {
            VideoNode::Branch(children) => {
                for child in &mut *children {
                    child.remove(entity);
                }

                children.retain(
                    |child| !matches!(child, VideoNode::Branch(children) if children.is_empty()),
                );

                if let [child] = children.as_mut_slice() {
                    *self = mem::take(child);
                }
            }
            VideoNode::Leaf(this) => {
                if entity == *this {
                    *self = VideoNode::Branch(vec![]);
                }
            }
        }
    }

    fn count_children(&self) -> u32 {
        match self {
            VideoNode::Branch(children) => children.iter().map(|it| it.count_children()).sum(),
            VideoNode::Leaf(_) => 1,
        }
    }

    fn max_depth(&self) -> u32 {
        match self {
            VideoNode::Branch(children) => {
                1 + children.iter().map(|it| it.max_depth()).max().unwrap_or(0)
            }
            VideoNode::Leaf(_) => 1,
        }
    }
}

impl VideoLayout {
    fn opposite(&self) -> Self {
        match self {
            VideoLayout::Horizontal => VideoLayout::Vertical,
            VideoLayout::Vertical => VideoLayout::Horizontal,
        }
    }
}

impl Default for VideoNode {
    fn default() -> Self {
        VideoNode::Branch(vec![])
    }
}

#[derive(Component)]
struct DisplayCamera;
#[derive(Component)]
struct DisplayParent;
#[derive(Component)]
struct DisplayMarker;
#[derive(Component)]
struct VideoFeedDisplay;

#[derive(Resource, Default)]
pub struct VideoDisplay2DSettings {
    pub enabled: bool,
}

fn setup(mut cmds: Commands) {
    let camera = cmds
        .spawn((
            Camera2dBundle {
                camera: BevyCamera {
                    is_active: false,
                    ..default()
                },
                ..default()
            },
            DisplayCamera,
            RENDER_LAYERS,
        ))
        .id();

    // Root
    cmds.spawn((
        Name::new("Cameras 2D"),
        root(VideoLayout::default()),
        TargetCamera(camera),
        VideoTree::default(),
        DisplayParent,
    ));
}

fn create_display(
    mut cmds: Commands,

    new_cameras: Query<Entity, (With<Camera>, Added<Handle<Image>>)>,
    mut lost_cameras: RemovedComponents<Camera>,

    cameras: Query<&Handle<Image>>,
    mut parent: Query<(Entity, &mut VideoTree), With<DisplayParent>>,
) {
    let (parent, mut tree) = parent.single_mut();
    let mut tree_changed = false;

    for entity in &new_cameras {
        tree.0.insert(entity);
        tree_changed = true;
    }

    for entity in lost_cameras.read() {
        tree.0.remove(entity);
        tree_changed = true;
    }

    if tree_changed {
        let layout = VideoLayout::default();
        let depth = tree.0.max_depth() as i32 - 1;

        let size_hint = match layout {
            VideoLayout::Horizontal => (
                0.5f32.powi(depth / 2 + depth % 2) * 100.0,
                0.5f32.powi(depth / 2) * 100.0,
            ),
            VideoLayout::Vertical => (
                0.5f32.powi(depth / 2) * 100.0,
                0.5f32.powi(depth / 2 + depth % 2) * 100.0,
            ),
        };

        cmds.entity(parent)
            .despawn_descendants()
            .with_children(move |builder| {
                builder.spawn(root(layout)).with_children(|builder| {
                    build_tree(builder, &tree.0, &cameras, layout, size_hint);
                });
            });
    }
}

// FIXME: Approch in display_3d is a bit cleaner and perhaps more efficient
fn update_aspect_ratio(
    mut displays: Query<(&mut Style, &UiImage), With<VideoFeedDisplay>>,
    mut image_events: EventReader<AssetEvent<Image>>,
    images: Res<Assets<Image>>,
) {
    for image_event in image_events.read() {
        if let AssetEvent::Added { id } | AssetEvent::Modified { id } = image_event {
            for (mut style, image) in &mut displays {
                let handle = &image.texture;

                if handle.id() == *id {
                    let aspect_ratio = images
                        .get(handle)
                        .map(|it| f32::from(it.aspect_ratio()))
                        .unwrap_or(16.0 / 9.0);

                    // We dont want to unnecessarially trigger anyone's change detection
                    if style.aspect_ratio != Some(aspect_ratio) {
                        style.aspect_ratio = Some(aspect_ratio);
                    }
                }
            }
        }
    }
}

fn build_tree(
    builder: &mut ChildBuilder,
    tree: &VideoNode,
    cameras: &Query<&Handle<Image>>,
    layout: VideoLayout,
    size_hint: (f32, f32),
) {
    match tree {
        VideoNode::Branch(children) => {
            #[derive(Clone, Copy)]
            enum ChildType<'a> {
                Tree(&'a VideoNode),
                Seprator,
            }

            for child in children
                .iter()
                .map(ChildType::Tree)
                .intersperse(ChildType::Seprator)
            {
                match child {
                    ChildType::Tree(node) => {
                        let child_layout = layout.opposite();

                        builder
                            .spawn(subroot(child_layout))
                            .with_children(|builder| {
                                build_tree(builder, node, cameras, child_layout, size_hint)
                            });
                    }
                    ChildType::Seprator => {
                        builder.spawn(separator(layout));
                    }
                }
            }
        }
        VideoNode::Leaf(camera_entity) => {
            let weak_texture = cameras
                .get(*camera_entity)
                .map(|it| it.clone_weak())
                .unwrap_or_else(|_| Default::default());

            builder
                .spawn(container(layout))
                // TODO: video feed image
                .with_children(|builder| {
                    builder.spawn(feed(layout, weak_texture, size_hint));
                });
        }
    }
}

fn enable_camera(
    mut last: Local<bool>,
    mut camera: Query<&mut BevyCamera, With<DisplayCamera>>,
    settings: Res<VideoDisplay2DSettings>,
) {
    if *last != settings.enabled {
        for mut camera in camera.iter_mut() {
            camera.is_active = settings.enabled;
        }

        *last = settings.enabled;
    }
}

fn root(layout: VideoLayout) -> impl Bundle {
    match layout {
        VideoLayout::Horizontal => (
            NodeBundle {
                style: Style {
                    width: Val::Vw(100.0),
                    height: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    flex_direction: FlexDirection::Row,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::BLUE)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
        VideoLayout::Vertical => (
            NodeBundle {
                style: Style {
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    flex_direction: FlexDirection::Column,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::BLUE)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
    }
}

fn subroot(layout: VideoLayout) -> impl Bundle {
    match layout {
        VideoLayout::Horizontal => (
            NodeBundle {
                style: Style {
                    flex_grow: 1.0,
                    // display: Display::Grid,
                    min_height: Val::Px(0.0),
                    height: Val::Percent(100.0),
                    width: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    flex_direction: FlexDirection::Row,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::ORANGE)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
        VideoLayout::Vertical => (
            NodeBundle {
                style: Style {
                    flex_grow: 1.0,
                    // display: Display::Grid,
                    min_width: Val::Px(0.0),
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    flex_direction: FlexDirection::Column,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::ORANGE)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
    }
}

fn container(layout: VideoLayout) -> impl Bundle {
    match layout {
        VideoLayout::Horizontal => (
            NodeBundle {
                style: Style {
                    flex_grow: 1.0,
                    height: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::SpaceEvenly,
                    flex_direction: FlexDirection::Row,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::GREEN)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
        VideoLayout::Vertical => (
            NodeBundle {
                style: Style {
                    flex_grow: 1.0,
                    width: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::SpaceEvenly,
                    flex_direction: FlexDirection::Row,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::GREEN)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
    }
}

fn feed(layout: VideoLayout, texture: Handle<Image>, size_hint: (f32, f32)) -> impl Bundle {
    match layout {
        VideoLayout::Horizontal => (
            ImageBundle {
                style: Style {
                    // height: Val::Percent(100.0),
                    // width: Val::Vw(size_hint.0),
                    // height: Val::Vh(size_hint.1),
                    max_width: Val::Vw(size_hint.0),
                    max_height: Val::Vw(size_hint.1),
                    flex_direction: FlexDirection::Row,
                    aspect_ratio: Some(16.0 / 9.0),
                    ..default()
                },
                // background_color: BackgroundColor(Color::PINK),
                image: UiImage::new(texture),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
            VideoFeedDisplay,
        ),
        VideoLayout::Vertical => (
            ImageBundle {
                style: Style {
                    // width: Val::Percent(100.0),
                    // width: Val::Vw(size_hint.0),
                    // height: Val::Vh(size_hint.1),
                    max_width: Val::Vw(size_hint.0),
                    max_height: Val::Vh(size_hint.1),
                    flex_direction: FlexDirection::Row,
                    aspect_ratio: Some(16.0 / 9.0),
                    ..default()
                },
                // background_color: BackgroundColor(Color::PINK),
                image: UiImage::new(texture),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
            VideoFeedDisplay,
        ),
    }
}

fn separator(layout: VideoLayout) -> impl Bundle {
    match layout {
        VideoLayout::Horizontal => (
            NodeBundle {
                style: Style {
                    height: Val::Px(25.0),
                    width: Val::Px(5.0),
                    flex_direction: FlexDirection::Row,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::PURPLE)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
        VideoLayout::Vertical => (
            NodeBundle {
                style: Style {
                    height: Val::Px(5.0),
                    width: Val::Px(25.0),
                    flex_direction: FlexDirection::Row,
                    ..default()
                },
                background_color: BackgroundColor(Color::from(css::PURPLE)),
                ..default()
            },
            RENDER_LAYERS,
            DisplayMarker,
        ),
    }
}
