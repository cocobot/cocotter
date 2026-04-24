//! Bevy app: hosts the IPC listener in a background thread, mirrors world
//! entities (robots + humans) into ECS entities, and renders them in 3D
//! unless `--headless`.

use std::collections::HashMap;
use std::f32::consts::FRAC_PI_4;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::asset::RenderAssetUsages;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::post_process::bloom::Bloom;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_mod_outline::{
    AsyncSceneInheritOutline, AutoGenerateOutlineNormalsPlugin, GenerateOutlineNormalsSettings,
    OutlinePlugin, OutlineVolume,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

use sim_protocol::{Pose2D, RobotKind};

use crate::bridge::WorldUpdate;
use crate::config::{CollisionPrimitive, Config, Finish, NeopixelFixture};
use crate::controls::{
    self, ChordStateRes, ConnRegistry, SpawnSlots,
};
use crate::textures;
use crate::world::EntityKind;

#[derive(Component)]
struct CollisionOverlay;

/// Tag for a single simulated neopixel (sphere child of a robot entity).
#[derive(Component)]
struct NeopixelLed;

/// Per-robot registry of `(entity, material-handle)` pairs in strip
/// order. `NeopixelFrame` updates look up `robot_id` here and stream
/// colours straight into the handles; the entity is used to hide the
/// sphere when the LED is off so it doesn't render as a black ball.
#[derive(Resource, Default)]
struct NeopixelRegistry(HashMap<String, Vec<(Entity, Handle<StandardMaterial>)>>);

/// Runtime toggle for the human (spectator) visual entities. They always
/// exist in the sim world (for lidar realism) but the visuals can be
/// hidden with the `C` shortcut.
#[derive(Resource)]
struct HumansVisible(bool);

/// Convert millimeters (protocol / sim world) to meters (Bevy scene).
const MM: f32 = 0.001;

#[derive(Resource)]
struct BridgeRx(flume::Receiver<WorldUpdate>);

#[derive(Resource, Default)]
struct SimEntities(HashMap<String, (Entity, EntityInfo)>);

struct EntityInfo {
    kind: EntityKind,
    /// Bbox half-height in meters. For robots this is rendered above the
    /// table top; for humans it's rendered above the floor.
    half_height_m: f32,
    /// Whether the mesh's origin is at the base (true, glTF characters)
    /// or at the centre (false, Bevy primitives).
    bottom_anchored: bool,
}

#[derive(Resource)]
struct HumanAssets {
    scene: Handle<Scene>,
    animation_graph: Handle<AnimationGraph>,
    walk_node: AnimationNodeIndex,
    /// CesiumMan is ~1.0 m tall in its default pose; scale to match the
    /// configured body height.
    native_height_m: f32,
}

#[derive(Resource, Default)]
struct RobotVisualAssets {
    galipeur: Option<Handle<Scene>>,
    pami: Option<Handle<Scene>>,
}

impl RobotVisualAssets {
    fn for_kind(&self, kind: RobotKind) -> Option<&Handle<Scene>> {
        match kind {
            RobotKind::Galipeur => self.galipeur.as_ref(),
            RobotKind::Pami => self.pami.as_ref(),
        }
    }
}

#[derive(Resource)]
struct SimConfig(Config);

#[derive(Resource)]
struct Headless(bool);

#[derive(Component)]
struct SimEntityTag {
    #[allow(dead_code)]
    id: String,
    kind: EntityKind,
}

pub fn run(
    config: Config,
    bridge_rx: flume::Receiver<WorldUpdate>,
    headless: bool,
    conn_registry: ConnRegistry,
) {
    let mut app = App::new();
    app.insert_resource(BridgeRx(bridge_rx))
        .insert_resource(SimEntities::default())
        .insert_resource(SimConfig(config))
        .insert_resource(Headless(headless))
        .insert_resource(conn_registry)
        .insert_resource(ChordStateRes::default())
        .insert_resource(SpawnSlots::default())
        .insert_resource(HumansVisible(false))
        .insert_resource(NeopixelRegistry::default());

    if headless {
        // Run a fixed-tick loop with no rendering.
        app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(
            Duration::from_secs_f32(1.0 / 60.0),
        )));
    } else {
        app.add_plugins((
            DefaultPlugins
                .set(AssetPlugin {
                    file_path: format!("{}/assets", env!("CARGO_MANIFEST_DIR")),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "meca_cocotter sim".into(),
                        resolution: (1200u32, 800u32).into(),
                        ..default()
                    }),
                    ..default()
                }),
            // LMB drag = orbit · RMB drag = pan · wheel = zoom · R = reset
            PanOrbitCameraPlugin,
            // Cartoon-style outline pass (inverted-hull technique).
            OutlinePlugin,
            AutoGenerateOutlineNormalsPlugin::new(GenerateOutlineNormalsSettings::default()),
        ));
    }

    app.add_systems(
        Startup,
        (
            setup_world,
            setup_camera_and_light,
            setup_human_assets,
            setup_robot_assets,
            setup_shortcuts_overlay,
        ),
    )
    .add_systems(
        Update,
        (
            drain_bridge,
            start_human_animations,
            toggle_collision_overlay,
            toggle_humans_visibility,
            apply_humans_visibility,
            ensure_mesh_normals,
            controls::chord_input,
            controls::render_chord_overlay,
        ),
    );

    app.run();
}

fn setup_human_assets(
    mut commands: Commands,
    asset_server: Option<Res<AssetServer>>,
    graphs: Option<ResMut<Assets<AnimationGraph>>>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let Some(asset_server) = asset_server else { return };
    let Some(mut graphs) = graphs else { return };

    let scene: Handle<Scene> = asset_server.load("humans/cesium_man.glb#Scene0");
    let clip = asset_server.load("humans/cesium_man.glb#Animation0");
    let (graph, walk_node) = AnimationGraph::from_clip(clip);
    let animation_graph = graphs.add(graph);

    commands.insert_resource(HumanAssets {
        scene,
        animation_graph,
        walk_node,
        native_height_m: 1.0,
    });
}

/// Attach the animation graph + play the walk clip on any `AnimationPlayer`
/// that Bevy spawned inside a CesiumMan scene this frame.
fn start_human_animations(
    mut commands: Commands,
    assets: Option<Res<HumanAssets>>,
    mut players: Query<(Entity, &mut AnimationPlayer), Added<AnimationPlayer>>,
) {
    let Some(assets) = assets else { return };
    for (entity, mut player) in &mut players {
        commands
            .entity(entity)
            .insert(AnimationGraphHandle(assets.animation_graph.clone()));
        player.play(assets.walk_node).repeat();
    }
}

fn setup_robot_assets(
    mut commands: Commands,
    asset_server: Option<Res<AssetServer>>,
    config: Res<SimConfig>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let Some(asset_server) = asset_server else { return };

    let load = |cfg: &crate::config::RobotConfig| -> Option<Handle<Scene>> {
        let visual = cfg.model.as_ref()?.visual.as_deref()?;
        // Asset paths are resolved against the AssetPlugin `file_path`
        // (configured to `sim/assets/`), but our TOML stores workspace-
        // relative paths like `sim/assets/robots/galipeur.glb`. Strip the
        // `sim/assets/` prefix so the asset loader can find it.
        let key = visual
            .strip_prefix("sim/assets/")
            .unwrap_or(visual);
        Some(asset_server.load(format!("{}#Scene0", key)))
    };

    commands.insert_resource(RobotVisualAssets {
        galipeur: load(&config.0.galipeur),
        pami: load(&config.0.pami),
    });
}

fn stand_height_m(config: &Config) -> f32 {
    config.field.stand_height_mm * MM
}

fn setup_world(
    mut commands: Commands,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
    images: Option<ResMut<Assets<Image>>>,
    config: Res<SimConfig>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let (Some(mut meshes), Some(mut materials), Some(mut images)) =
        (meshes, materials, images)
    else {
        return;
    };

    let stand_h = stand_height_m(&config.0);
    let field_w_m = config.0.field.width_mm as f32 * MM;
    let field_d_m = config.0.field.height_mm as f32 * MM;

    // Table stand (visual only, rendered from y=0 up to y=stand_h).
    if stand_h > 1e-4 {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(field_w_m, stand_h, field_d_m))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.33, 0.30),
                perceptual_roughness: 0.95,
                ..default()
            })),
            Transform::from_xyz(field_w_m * 0.5, stand_h * 0.5, field_d_m * 0.5),
        ));

        // Floor around the stand (big grey plane so humans don't look like
        // they're walking on the void).
        let floor_side = (field_w_m + field_d_m) * 3.0;
        commands.spawn((
            Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(floor_side * 0.5, floor_side * 0.5)))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.12, 0.12, 0.13),
                perceptual_roughness: 1.0,
                ..default()
            })),
            Transform::from_xyz(field_w_m * 0.5, 0.0, field_d_m * 0.5),
        ));
    }

    // Every table obstacle — including the flat ground — goes through the
    // same rendering path, offset up by the stand height.
    let mut texture_cache: std::collections::HashMap<String, Handle<Image>> =
        std::collections::HashMap::new();

    let field_w = config.0.field.width_mm as f32;
    let field_h = config.0.field.height_mm as f32;
    let playmat_handle: Option<Handle<Image>> = config
        .0
        .field
        .playmat
        .as_ref()
        .and_then(|pm| match textures::load_png(std::path::Path::new(&pm.path)) {
            Ok(img) => Some(images.add(img)),
            Err(e) => {
                log::error!("playmat {} load failed: {e}", pm.path);
                None
            }
        });

    for obs in &config.0.field.obstacles {
        let [x0, y0, x1, y1] = obs.aabb_mm;
        let w = ((x1 - x0) * MM).abs();
        let d = ((y1 - y0) * MM).abs();
        let h = obs.height_mm * MM;
        if w < 1e-4 || d < 1e-4 {
            continue;
        }
        let cx = (x0 + x1) * 0.5 * MM;
        let cy = (y0 + y1) * 0.5 * MM;

        // Material for the body (sides + bottom). The playmat is *not*
        // used here even when `use_playmat = true` — it only belongs on
        // the top face, and gets spawned as a separate plane below.
        // Precedence for the body: explicit texture > finish > flat color.
        let body_material = if let Some(path) = obs.texture.as_deref() {
            let handle = texture_cache
                .entry(path.to_string())
                .or_insert_with(|| match textures::load_png(std::path::Path::new(path)) {
                    Ok(img) => images.add(img),
                    Err(e) => {
                        log::error!("texture {} load failed: {e}", path);
                        Handle::<Image>::default()
                    }
                })
                .clone();
            StandardMaterial {
                base_color: Color::WHITE,
                base_color_texture: Some(handle),
                perceptual_roughness: 0.9,
                ..default()
            }
        } else {
            let rgb = obs.srgb_color();
            match obs.finish {
                Finish::Smooth => StandardMaterial {
                    base_color: Color::srgb(rgb[0], rgb[1], rgb[2]),
                    perceptual_roughness: 0.9,
                    ..default()
                },
                Finish::Wood => {
                    let tex = textures::wood_grain(rgb, 256, 512);
                    StandardMaterial {
                        base_color: Color::WHITE,
                        base_color_texture: Some(images.add(tex)),
                        perceptual_roughness: 0.95,
                        ..default()
                    }
                }
            }
        };

        // `height_mm = 0` marks the ground — physics/raycast skip it, but
        // we still give it a 1 mm visual thickness so it renders like any
        // other obstacle (same cuboid path, playmat visible on top).
        const GROUND_VISUAL_H_M: f32 = 0.001;
        let visual_h = if h < 1e-4 { GROUND_VISUAL_H_M } else { h };
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(w, visual_h, d))),
            MeshMaterial3d(materials.add(body_material)),
            Transform::from_xyz(cx, stand_h + visual_h * 0.5, cy),
        ));

        // Playmat decal on the top face. The UV transform crops the
        // region of the field-wide image corresponding to this obstacle's
        // AABB, so a playmat tile on top of `granary_ground` shows the
        // correct portion of the playing area.
        if obs.use_playmat {
            if let Some(handle) = playmat_handle.as_ref() {
                let scale_u = (x1 - x0) / field_w;
                let scale_v = (y1 - y0) / field_h;
                let trans_u = x0 / field_w;
                let trans_v = y0 / field_h;
                let pm_material = StandardMaterial {
                    base_color: Color::WHITE,
                    base_color_texture: Some(handle.clone()),
                    perceptual_roughness: 0.9,
                    uv_transform: bevy::math::Affine2::from_scale_angle_translation(
                        Vec2::new(scale_u, scale_v),
                        0.0,
                        Vec2::new(trans_u, trans_v),
                    ),
                    ..default()
                };
                commands.spawn((
                    Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(w * 0.5, d * 0.5)))),
                    MeshMaterial3d(materials.add(pm_material)),
                    Transform::from_xyz(cx, stand_h + visual_h + 0.001, cy),
                ));
            }
        }
    }
}

fn setup_shortcuts_overlay(mut commands: Commands, headless: Res<Headless>) {
    if headless.0 {
        return;
    }
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(6.0),
                left: Val::Px(8.0),
                padding: UiRect::axes(Val::Px(8.0), Val::Px(4.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(
                    "LMB orbit | RMB pan | wheel zoom  |  \
                     H collision overlay  |  C toggle humans  |  \
                     S spawn (->G/P->B/Y)  |  K kill-all  |  \
                     Q quit  |  Esc cancel",
                ),
                TextFont { font_size: 12.0, ..default() },
                TextColor(Color::srgba(1.0, 1.0, 1.0, 0.9)),
            ));
        });
}

fn setup_camera_and_light(
    mut commands: Commands,
    config: Res<SimConfig>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let w = config.0.field.width_mm as f32 * MM;
    let h = config.0.field.height_mm as f32 * MM;
    let stand_h = stand_height_m(&config.0);
    let cx = w / 2.0;
    let cy = h / 2.0;

    // Initial framing: "3/4 view" at pitch ≈ 35°, distance ≈ 1.2 × diagonal.
    // PanOrbitCamera takes over afterwards — LMB drag rotates around the
    // focus, RMB drag pans it, wheel zooms.
    let table_diag = (w * w + h * h).sqrt();
    let focus = Vec3::new(cx, stand_h + 0.1, cy);
    commands.spawn((
        Camera3d::default(),
        // Tonemapping + bloom make the emissive neopixels actually glow
        // instead of looking like painted marbles. Bevy 0.18's bloom
        // pass already reads the HDR pipeline automatically.
        Tonemapping::TonyMcMapface,
        Bloom::NATURAL,
        PanOrbitCamera {
            focus,
            radius: Some(table_diag * 1.2),
            pitch: Some(35.0_f32.to_radians()),
            yaw: Some(0.0),
            ..default()
        },
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(cx, stand_h + 5.0, cy)
            .with_rotation(Quat::from_rotation_x(-FRAC_PI_4)),
    ));
}

fn drain_bridge(
    mut commands: Commands,
    bridge: Res<BridgeRx>,
    mut registry: ResMut<SimEntities>,
    mut transforms: Query<(&mut Transform, &SimEntityTag)>,
    mut leds: Query<(&mut Visibility, &mut SpotLight), (With<NeopixelLed>, Without<SimEntityTag>)>,
    config: Res<SimConfig>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
    human_assets: Option<Res<HumanAssets>>,
    robot_assets: Option<Res<RobotVisualAssets>>,
    humans_visible: Res<HumansVisible>,
    mut neopixels: ResMut<NeopixelRegistry>,
    headless: Res<Headless>,
) {
    let stand_h = stand_height_m(&config.0);

    // Mesh/material handles are only available when DefaultPlugins provided
    // the asset + rendering plugins; in headless mode we only track the
    // Transform component and skip rendering.
    let mut visuals = match (headless.0, meshes, materials) {
        (false, Some(m), Some(mat)) => Some((m, mat)),
        _ => None,
    };

    while let Ok(update) = bridge.0.try_recv() {
        match update {
            WorldUpdate::Spawn { id, kind, pose, width_mm, length_mm, body_height_mm } => {
                if registry.0.contains_key(&id) {
                    continue;
                }
                let h_m = body_height_mm * MM;
                let robot_scene: Option<Handle<Scene>> = match kind {
                    EntityKind::Robot(rk) => robot_assets
                        .as_ref()
                        .and_then(|ra| ra.for_kind(rk).cloned()),
                    _ => None,
                };
                let is_human_scene = matches!(kind, EntityKind::Human) && human_assets.is_some();
                let is_scene = is_human_scene || robot_scene.is_some();
                let info = EntityInfo {
                    kind,
                    half_height_m: h_m * 0.5,
                    bottom_anchored: is_scene,
                };
                let t = pose_to_transform(&pose, &info, stand_h);
                // Humans spawn asynchronously after the resource-change
                // window of `apply_humans_visibility` has passed, so set
                // their initial `Visibility` directly here.
                let initial_vis = match kind {
                    EntityKind::Human if !humans_visible.0 => Visibility::Hidden,
                    _ => Visibility::default(),
                };
                // `Visibility` is needed so Bevy propagates it to spawned
                // children (SceneRoot, collision overlays). Without it,
                // Bevy 0.18 warns B0004 and children don't update their
                // InheritedVisibility.
                let mut cmd = commands.spawn((
                    SimEntityTag { id: id.clone(), kind },
                    t,
                    initial_vis,
                ));

                if let Some((ref mut meshes, ref mut materials)) = visuals {
                    let (w_m, l_m) = (width_mm * MM, length_mm * MM);
                    let color = match kind {
                        EntityKind::Robot(RobotKind::Galipeur) => Color::srgb(0.2, 0.6, 1.0),
                        EntityKind::Robot(RobotKind::Pami) => Color::srgb(1.0, 0.6, 0.2),
                        EntityKind::Human => Color::srgb(0.85, 0.75, 0.65),
                    };
                    match kind {
                        EntityKind::Human if human_assets.is_some() => {
                            // Real humanoid scene from glTF, scaled to the
                            // configured body height.
                            let ha = human_assets.as_ref().unwrap();
                            let scale = h_m / ha.native_height_m;
                            cmd.insert(SceneRoot(ha.scene.clone()));
                            if let Ok((mut t_mut, _)) = transforms.get_mut(cmd.id()) {
                                t_mut.scale = Vec3::splat(scale);
                            } else {
                                // Re-spawn transform with scale baked in.
                                let mut t2 = t;
                                t2.scale = Vec3::splat(scale);
                                cmd.insert(t2);
                            }
                        }
                        EntityKind::Human => {
                            // Fallback capsule.
                            let r = (w_m.min(l_m) * 0.5).min(h_m * 0.45).max(0.05);
                            let cyl = (h_m - 2.0 * r).max(0.01);
                            cmd.insert((
                                Mesh3d(meshes.add(Capsule3d::new(r, cyl))),
                                MeshMaterial3d(materials.add(StandardMaterial {
                                    base_color: color,
                                    perceptual_roughness: 0.9,
                                    ..default()
                                })),
                            ));
                        }
                        EntityKind::Robot(rk) if robot_scene.is_some() => {
                            // Spawn the scene as a child so we can apply a
                            // per-model translation (wheel axle → chassis
                            // base) without fighting the parent's pose.
                            let offset = match rk {
                                RobotKind::Galipeur => config
                                    .0
                                    .galipeur
                                    .model
                                    .as_ref()
                                    .map(|m| m.visual_offset_mm)
                                    .unwrap_or([0.0; 3]),
                                RobotKind::Pami => config
                                    .0
                                    .pami
                                    .model
                                    .as_ref()
                                    .map(|m| m.visual_offset_mm)
                                    .unwrap_or([0.0; 3]),
                            };
                            let scene_handle = robot_scene.clone().unwrap();
                            cmd.with_children(|p| {
                                p.spawn((
                                    SceneRoot(scene_handle),
                                    Transform::from_xyz(
                                        offset[0] * MM,
                                        offset[1] * MM,
                                        offset[2] * MM,
                                    ),
                                    // Black ink-style outline on every
                                    // mesh in the glTF scene. `AsyncScene…`
                                    // waits for the scene to load, then
                                    // sprinkles `InheritOutline` onto
                                    // every child so the inverted-hull
                                    // pass is applied mesh-by-mesh.
                                    OutlineVolume {
                                        visible: true,
                                        width: 2.0,
                                        colour: Color::BLACK,
                                    },
                                    AsyncSceneInheritOutline::default(),
                                ));
                            });
                        }
                        EntityKind::Robot(_) => {
                            cmd.insert((
                                Mesh3d(meshes.add(Cuboid::new(l_m, h_m.max(0.02), w_m))),
                                MeshMaterial3d(materials.add(StandardMaterial {
                                    base_color: color,
                                    perceptual_roughness: 0.9,
                                    ..default()
                                })),
                            ));
                        }
                    }

                    // Attach collision overlay children for robots configured
                    // with a simplified model. Hidden by default; toggle with
                    // the H key.
                    if let EntityKind::Robot(rk) = kind {
                        let prims: Option<&Vec<CollisionPrimitive>> = match rk {
                            RobotKind::Galipeur => config
                                .0
                                .galipeur
                                .model
                                .as_ref()
                                .map(|m| &m.collision),
                            RobotKind::Pami => config
                                .0
                                .pami
                                .model
                                .as_ref()
                                .map(|m| &m.collision),
                        };
                        // Parent entity sits at the visual's own anchor:
                        // scenes are bottom-anchored, cuboids are centre-
                        // anchored. Shift accordingly for body-frame Z
                        // to line up with the table top at y=0.
                        let y_off = if is_scene { 0.0 } else { -info.half_height_m };
                        if let Some(prims) = prims {
                            if !prims.is_empty() {
                                let children =
                                    build_overlay_spawns(prims, y_off, meshes, materials);
                                cmd.with_children(|parent| {
                                    for bundle in children {
                                        parent.spawn(bundle);
                                    }
                                });
                            }
                        }

                        // Neopixel fixtures, if any. Each LED is a small
                        // sphere child; we keep a Vec<Handle<Material>>
                        // in strip order so incoming `NeopixelFrame`s can
                        // recolour them directly.
                        let fixtures: &[NeopixelFixture] = match rk {
                            RobotKind::Galipeur => &config.0.galipeur.neopixels,
                            RobotKind::Pami => &config.0.pami.neopixels,
                        };
                        if !fixtures.is_empty() {
                            let led_bundles =
                                build_neopixel_spawns(fixtures, y_off, meshes, materials);
                            let mut entries = Vec::with_capacity(led_bundles.len());
                            cmd.with_children(|parent| {
                                for bundle in led_bundles {
                                    let mat_handle = bundle.1 .0.clone();
                                    let e = parent.spawn(bundle).id();
                                    entries.push((e, mat_handle));
                                }
                            });
                            neopixels.0.insert(id.clone(), entries);
                        }
                    }
                }
                let entity = cmd.id();
                registry.0.insert(id, (entity, info));
                log::info!("[sim-app] spawned {:?}", kind);
            }
            WorldUpdate::UpdatePose { id, pose } => {
                if let Some((entity, info)) = registry.0.get(&id) {
                    if let Ok((mut t, _)) = transforms.get_mut(*entity) {
                        *t = pose_to_transform(&pose, info, stand_h);
                    }
                }
            }
            WorldUpdate::Despawn { id } => {
                if let Some((entity, _)) = registry.0.remove(&id) {
                    commands.entity(entity).despawn();
                }
                neopixels.0.remove(&id);
            }
            WorldUpdate::Neopixels { id, pixels } => {
                let Some(entries) = neopixels.0.get(&id) else { continue };
                let Some((_, ref mut materials_ref)) = visuals else { continue };
                // Gain on emissive pushes the die into HDR so bloom
                // picks it up (you see a halo around the die).
                const EMISSIVE_GAIN: f32 = 20.0;
                // Lumens at full brightness (255). ~200 lm is a strong
                // flashlight, fits a small 5mm LED die used on a robot.
                const SPOT_LUMENS_FULL: f32 = 200.0;
                for (i, rgb) in pixels.iter().enumerate() {
                    let Some((entity, handle)) = entries.get(i) else { break };
                    let r = rgb[0] as f32 / 255.0;
                    let g = rgb[1] as f32 / 255.0;
                    let b = rgb[2] as f32 / 255.0;
                    let brightness = r.max(g).max(b);
                    let on = brightness > 1.0 / 255.0;
                    if let Ok((mut vis, mut light)) = leds.get_mut(*entity) {
                        if on {
                            *vis = Visibility::Inherited;
                            light.color = Color::srgb(r, g, b);
                            light.intensity = brightness * SPOT_LUMENS_FULL;
                        } else {
                            *vis = Visibility::Hidden;
                            light.intensity = 0.0;
                        }
                    }
                    if on {
                        if let Some(mat) = materials_ref.get_mut(handle) {
                            mat.base_color = Color::srgb(r, g, b);
                            mat.emissive = LinearRgba::new(
                                r * EMISSIVE_GAIN,
                                g * EMISSIVE_GAIN,
                                b * EMISSIVE_GAIN,
                                1.0,
                            );
                        }
                    }
                }
            }
        }
    }
}

type OverlaySpawn = (
    Mesh3d,
    MeshMaterial3d<StandardMaterial>,
    Transform,
    Visibility,
    CollisionOverlay,
);

type LedSpawn = (
    Mesh3d,
    MeshMaterial3d<StandardMaterial>,
    Transform,
    Visibility,
    SpotLight,
    NeopixelLed,
);

fn neopixel_material() -> StandardMaterial {
    // Unlit: the die is a pure emitter. Starts black; the robot's frame
    // overwrites base_color + emissive. Bloom on the camera turns the
    // emissive into a real glow around the die.
    StandardMaterial {
        base_color: Color::BLACK,
        emissive: LinearRgba::BLACK,
        unlit: true,
        ..default()
    }
}

fn body_pos_to_bevy(pos_mm: [f32; 3], parent_y_off_m: f32) -> Vec3 {
    // Body frame is (x_forward, y_left, z_up). The robot entity maps
    // sim x -> Bevy x and sim y -> Bevy z (Bevy y is up).
    Vec3::new(
        pos_mm[0] * MM,
        parent_y_off_m + pos_mm[2] * MM,
        pos_mm[1] * MM,
    )
}

/// Convert a body-frame unit direction `(x_fwd, y_left, z_up)` into
/// the same Bevy axis mapping used for positions.
fn body_dir_to_bevy(dir: [f32; 3]) -> Vec3 {
    Vec3::new(dir[0], dir[2], dir[1]).normalize_or_zero()
}

fn led_transform(pos_body_mm: [f32; 3], outward_body: [f32; 3], parent_y_off_m: f32) -> Transform {
    let pos = body_pos_to_bevy(pos_body_mm, parent_y_off_m);
    let fwd = body_dir_to_bevy(outward_body);
    // SpotLight's local forward is -Z. `looking_to` rotates so local -Z
    // aligns with `fwd`. `up` just needs to be non-parallel to fwd.
    let up = if fwd.y.abs() < 0.99 { Vec3::Y } else { Vec3::Z };
    Transform::from_translation(pos).looking_to(fwd, up)
}

fn build_neopixel_spawns(
    fixtures: &[NeopixelFixture],
    parent_y_offset_m: f32,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Vec<LedSpawn> {
    // Each LED starts off (intensity 0, colour black). The sim-tick
    // handler overwrites both when a frame arrives. Geometry: a small
    // sphere marks the die so the *source* is visible head-on; the
    // SpotLight on the same entity casts a tight, shadowed cone so the
    // volumetric fog can carve a beam through the air.
    const SPOT_INNER_DEG: f32 = 8.0;
    const SPOT_OUTER_DEG: f32 = 22.0;
    const SPOT_RANGE_M: f32 = 0.6;

    let mut out = Vec::new();
    let mut push = |pos_body: [f32; 3],
                    outward: [f32; 3],
                    pixel_size_mm: f32,
                    meshes: &mut ResMut<Assets<Mesh>>,
                    materials: &mut ResMut<Assets<StandardMaterial>>,
                    out: &mut Vec<LedSpawn>| {
        let r = pixel_size_mm * 0.5 * MM;
        let mesh = meshes.add(Sphere::new(r));
        let mat = materials.add(neopixel_material());
        out.push((
            Mesh3d(mesh),
            MeshMaterial3d(mat),
            led_transform(pos_body, outward, parent_y_offset_m),
            Visibility::Hidden,
            SpotLight {
                color: Color::BLACK,
                intensity: 0.0,
                range: SPOT_RANGE_M,
                radius: r,
                inner_angle: SPOT_INNER_DEG.to_radians(),
                outer_angle: SPOT_OUTER_DEG.to_radians(),
                shadows_enabled: false,
                ..default()
            },
            NeopixelLed,
        ));
    };

    for fixture in fixtures {
        match fixture {
            NeopixelFixture::Single {
                position_mm,
                pixel_size_mm,
                ..
            } => {
                // Default orientation for a lone pixel: body +Z (up).
                // Fine for status LEDs mounted on top; override by
                // moving position_mm if you need something else later.
                push(
                    *position_mm,
                    [0.0, 0.0, 1.0],
                    *pixel_size_mm,
                    meshes,
                    materials,
                    &mut out,
                );
            }
            NeopixelFixture::Ring {
                center_mm,
                radius_mm,
                count,
                start_angle_rad,
                pixel_size_mm,
                ..
            } => {
                let [cx, cy, cz] = *center_mm;
                for i in 0..*count {
                    let angle = start_angle_rad
                        + std::f32::consts::TAU * (i as f32) / (*count as f32);
                    let (ca, sa) = (angle.cos(), angle.sin());
                    let pos = [cx + radius_mm * ca, cy + radius_mm * sa, cz];
                    // Ring LEDs point radially outward in the body XY
                    // plane (Z-up stays zero).
                    let outward = [ca, sa, 0.0];
                    push(
                        pos,
                        outward,
                        *pixel_size_mm,
                        meshes,
                        materials,
                        &mut out,
                    );
                }
            }
        }
    }
    out
}

fn build_overlay_spawns(
    primitives: &[CollisionPrimitive],
    parent_y_offset_m: f32,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Vec<OverlaySpawn> {
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.9, 0.15, 0.15, 0.35),
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        perceptual_roughness: 0.8,
        unlit: true,
        ..default()
    });

    let mut out = Vec::with_capacity(primitives.len());
    for p in primitives {
        match p {
            CollisionPrimitive::Polygon { points_mm, z_base_mm, height_mm } => {
                let mesh = build_polygon_prism(points_mm, *height_mm * MM);
                out.push((
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(mat.clone()),
                    Transform::from_xyz(0.0, parent_y_offset_m + z_base_mm * MM, 0.0),
                    Visibility::Hidden,
                    CollisionOverlay,
                ));
            }
            CollisionPrimitive::Cylinder { center_mm, radius_mm, z_base_mm, height_mm } => {
                let h_m = *height_mm * MM;
                out.push((
                    Mesh3d(meshes.add(Cylinder::new(radius_mm * MM, h_m))),
                    MeshMaterial3d(mat.clone()),
                    Transform::from_xyz(
                        center_mm[0] * MM,
                        parent_y_offset_m + z_base_mm * MM + h_m * 0.5,
                        center_mm[1] * MM,
                    ),
                    Visibility::Hidden,
                    CollisionOverlay,
                ));
            }
        }
    }
    out
}

/// Build a triangle-soup mesh for a polygon extruded from y=0 to y=height_m.
/// Primitive points `[x_mm, y_mm]` map to Bevy local `(x*MM, 0, y*MM)` on the
/// bottom ring and `(x*MM, height_m, y*MM)` on the top ring. Winding assumes
/// the input polygon is ordered counter-clockwise in the robot's +X forward
/// / +Y left frame.
fn build_polygon_prism(points_mm: &[[f32; 2]], height_m: f32) -> Mesh {
    let n = points_mm.len();
    assert!(n >= 3, "polygon must have >= 3 points");

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(2 * n);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(2 * n);
    let mut indices: Vec<u32> = Vec::new();

    for &[x, z] in points_mm {
        positions.push([x * MM, 0.0, z * MM]);
        normals.push([0.0, -1.0, 0.0]);
    }
    for &[x, z] in points_mm {
        positions.push([x * MM, height_m, z * MM]);
        normals.push([0.0, 1.0, 0.0]);
    }

    let nu = n as u32;
    // Bottom fan (view from -Y → reverse winding so triangles face -Y).
    for i in 1..nu - 1 {
        indices.extend_from_slice(&[0, i + 1, i]);
    }
    // Top fan (view from +Y).
    for i in 1..nu - 1 {
        indices.extend_from_slice(&[nu, nu + i, nu + i + 1]);
    }
    // Sidewalls.
    for i in 0..nu {
        let next = (i + 1) % nu;
        indices.extend_from_slice(&[i, next, nu + next]);
        indices.extend_from_slice(&[i, nu + next, nu + i]);
    }

    // Keep the CPU copy (`MAIN_WORLD`) alongside the GPU one: outline
    // plugins run systems that inspect mesh attributes after extraction,
    // and they panic on RENDER_WORLD-only meshes.
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

/// Fill in `Mesh::ATTRIBUTE_NORMAL` on any freshly loaded mesh that lacks
/// it. `bevy_mod_outline`'s volume pipeline demands vertex normals, and
/// some glTF primitives (e.g. simplified/decimated exports) come through
/// without them — without this fix the outline queue spams
/// `Mesh is missing requested attribute: Vertex_Normal` every frame.
///
/// Only reacts to `Added` events and only upgrades to `get_mut` once the
/// read-only check has confirmed work is actually needed. `get_mut`
/// emits a `Modified` event even when no mutation happens, so touching
/// unrelated meshes here would turn into a per-frame feedback loop.
fn ensure_mesh_normals(
    mut meshes: Option<ResMut<Assets<Mesh>>>,
    mut events: MessageReader<AssetEvent<Mesh>>,
) {
    let Some(meshes) = meshes.as_mut() else { return };
    for event in events.read() {
        let AssetEvent::Added { id } = event else { continue };
        let needs_compute = {
            let Some(mesh) = meshes.get(*id) else { continue };
            mesh.primitive_topology() == PrimitiveTopology::TriangleList
                && mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some()
                && mesh.attribute(Mesh::ATTRIBUTE_NORMAL).is_none()
        };
        if !needs_compute {
            continue;
        }
        let Some(mesh) = meshes.get_mut(*id) else { continue };
        if let Err(e) = mesh.try_compute_normals() {
            log::warn!("ensure_mesh_normals: failed to compute normals: {e:?}");
        }
    }
}

fn toggle_collision_overlay(
    keys: Option<Res<ButtonInput<KeyCode>>>,
    mut q: Query<&mut Visibility, With<CollisionOverlay>>,
) {
    // MinimalPlugins (headless) doesn't provide a keyboard input resource.
    let Some(keys) = keys else { return };
    if keys.just_pressed(KeyCode::KeyH) {
        for mut v in &mut q {
            *v = match *v {
                Visibility::Hidden => Visibility::Visible,
                _ => Visibility::Hidden,
            };
        }
    }
}

fn toggle_humans_visibility(
    keys: Option<Res<ButtonInput<KeyCode>>>,
    mut visible: ResMut<HumansVisible>,
) {
    let Some(keys) = keys else { return };
    if keys.just_pressed(KeyCode::KeyC) {
        visible.0 = !visible.0;
    }
}

/// Push the `HumansVisible` flag down onto every human entity's
/// `Visibility`. Reacts to `is_changed()` so normal frames don't
/// scan the query.
fn apply_humans_visibility(
    visible: Res<HumansVisible>,
    mut q: Query<(&mut Visibility, &SimEntityTag)>,
) {
    if !visible.is_changed() {
        return;
    }
    let target = if visible.0 {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    for (mut v, tag) in &mut q {
        if matches!(tag.kind, EntityKind::Human) {
            *v = target;
        }
    }
}

fn pose_to_transform(pose: &Pose2D, info: &EntityInfo, stand_h: f32) -> Transform {
    // Robots sit on the table top; humans stand on the floor.
    let y_base = match info.kind {
        EntityKind::Robot(_) => stand_h,
        EntityKind::Human => 0.0,
    };
    // Primitive meshes are centred on their origin: shift up by half
    // height. glTF characters are bottom-anchored (origin at the feet).
    let y_offset = if info.bottom_anchored { 0.0 } else { info.half_height_m };

    // CesiumMan's "forward" in its local frame is +Z. Our protocol theta=0
    // means "facing world +X" (scene +X). The +π/2 pre-rotation aligns
    // the two, then `-theta` applies the walking heading. For primitive
    // meshes (symmetric cuboid / capsule) the pre-rotation is irrelevant.
    let heading_offset = if info.bottom_anchored { std::f32::consts::FRAC_PI_2 } else { 0.0 };
    Transform::from_xyz(pose.x_mm * MM, y_base + y_offset, pose.y_mm * MM)
        .with_rotation(Quat::from_rotation_y(heading_offset - pose.theta_rad))
}
