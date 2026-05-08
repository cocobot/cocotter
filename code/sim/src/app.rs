//! Bevy app: hosts the IPC listener in a background thread, mirrors world
//! entities (robots + humans) into ECS entities, and renders them in 3D
//! unless `--headless`.

use std::collections::HashMap;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::asset::RenderAssetUsages;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::pbr::{ScreenSpaceAmbientOcclusion, ScreenSpaceAmbientOcclusionQualityLevel};
use bevy::post_process::bloom::Bloom;
use bevy::render::renderer::RenderAdapterInfo;
use bevy::render::view::Hdr;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_mod_outline::{
    AsyncSceneInheritOutline, AutoGenerateOutlineNormalsPlugin, GenerateOutlineNormalsSettings,
    OutlinePlugin, OutlineVolume,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

use sim_protocol::{DebugVolume, DebugVolumeFrame, Pose2D, RobotKind};

use crate::bridge::WorldUpdate;
use crate::config::{CollisionPrimitive, Config, Finish, GroundLidar, NeopixelFixture};
use crate::controls::{
    self, AdversaryState, ChordStateRes, ConnRegistry, SharedWorld, SpawnSlots, TeamSidesRes,
    UpdatesTx,
};
use crate::textures;
use crate::world::{EntityKind, World};

#[derive(Component)]
struct CollisionOverlay;

/// Tag for a single simulated neopixel (sphere child of a robot entity).
#[derive(Component)]
pub struct NeopixelLed;

/// Tag for the visual cylinder of a ground lidar beam.
#[derive(Component)]
pub struct GroundLidarBeam;

/// Tag for the visual cylinder of one of the 12 rays in the LD06's
/// current packet slice. Toggled together via `Ld06BeamsVisible`
/// (`L` shortcut). 12 rays per robot, reused on every incoming packet.
#[derive(Component)]
pub struct Ld06Beam;

/// Tag for a translucent debug volume child entity. Toggled together
/// via `DebugVolumesVisible` (V shortcut).
#[derive(Component)]
struct DebugVolumeOverlay;

/// Bundle the three debug-volume params so `drain_bridge` stays under
/// Bevy's 16-parameter ceiling. Same trick as `JointsParam`.
#[derive(bevy::ecs::system::SystemParam)]
struct DebugVolumesParam<'w> {
    registry: ResMut<'w, DebugVolumeRegistry>,
    visible: Res<'w, DebugVolumesVisible>,
}

/// Bundle the LD06 viz params (registry + per-beam transform query +
/// runtime visibility flag) so they share a single slot in
/// `drain_bridge`. Same trick as `DebugVolumesParam`.
#[derive(bevy::ecs::system::SystemParam)]
struct Ld06Param<'w, 's> {
    registry: ResMut<'w, Ld06BeamRegistry>,
    visible: Res<'w, Ld06BeamsVisible>,
    beam_transforms: Query<
        'w,
        's,
        &'static mut Transform,
        (
            With<Ld06Beam>,
            Without<SimEntityTag>,
            Without<NeopixelLed>,
            Without<GroundLidarBeam>,
        ),
    >,
}

/// Bundle the rendering-side asset handles + per-robot scene assets
/// so they share a single slot in `drain_bridge`. Without this the
/// system overflows Bevy's 16-parameter limit once the joint and
/// debug-volume params are added.
#[derive(bevy::ecs::system::SystemParam)]
struct AssetsParam<'w> {
    meshes: Option<ResMut<'w, Assets<Mesh>>>,
    materials: Option<ResMut<'w, Assets<StandardMaterial>>>,
    human: Option<Res<'w, HumanAssets>>,
    robot: Option<Res<'w, RobotVisualAssets>>,
}

/// Per-beam geometry kept so incoming `GroundLidarHits` can clip the
/// cylinder in place: rebuild `translation = start + dir * hit/2` and
/// `scale.y = hit / max_range`.
#[derive(Clone, Copy)]
struct GroundBeamGeom {
    start_bevy: Vec3,
    dir_bevy: Vec3,
    max_range_m: f32,
}

#[derive(Resource, Default)]
struct GroundBeamRegistry(HashMap<String, Vec<(Entity, GroundBeamGeom)>>);

/// Per-robot list of the 12 LD06 ray entities (ordered as in the
/// packet). Each `Ld06Hits` event recomputes the 12 transforms in
/// place — the entities themselves persist for the life of the robot.
#[derive(Resource, Default)]
struct Ld06BeamRegistry(HashMap<String, [Entity; 12]>);

/// Runtime toggle for the LD06 beam viz. Hidden by default; press
/// `L` to render the rotor's current 12-ray slice.
#[derive(Resource)]
struct Ld06BeamsVisible(bool);

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

/// Runtime toggle for debug volumes (translucent boxes / cylinders the
/// robot pushes for in-sim diagnostics — face danger zones, swept
/// reach, etc.). Hidden by default; press `V` to show.
#[derive(Resource)]
struct DebugVolumesVisible(bool);

/// Per-robot list of debug-volume entities. Cleared and re-spawned
/// every time a fresh `WorldUpdate::DebugVolumes` arrives so the set
/// stays in lock-step with what the robot's mock pushed last.
#[derive(Resource, Default)]
struct DebugVolumeRegistry(HashMap<String, Vec<Entity>>);

/// Convert millimeters (protocol / sim world) to meters (Bevy scene).
pub const MM: f32 = 0.001;

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
            // Adversary uses a procedural stepped cylinder instead of a
            // glb scene — handled directly in the spawn match below.
            RobotKind::Adversary => None,
        }
    }
}

/// Shared sim config accessible to any Bevy system. Renamed from the
/// previous `SimConfig` so the chord/adversary code can spell it out
/// cleanly across modules; `controls.rs` reads field dimensions from
/// here when clamping the adversary inside the table.
#[derive(Resource)]
pub struct SimConfigRes(pub Config);

/// Filesystem path of the toml the sim was launched with. Stored so the
/// `R` reset handler can reload it on demand without a restart.
#[derive(Resource)]
pub struct ConfigPath(pub String);

/// Cross-thread handle to the live config. Bevy's `SimConfigRes` is a
/// fast read-only snapshot used by every system; the listener thread
/// reads through this lock at handshake time so reloads propagate to
/// freshly-spawned robots.
#[derive(Resource, Clone)]
pub struct SharedConfig(pub std::sync::Arc<std::sync::RwLock<Config>>);

#[derive(Resource)]
pub struct Headless(pub bool);

#[derive(Component)]
pub struct SimEntityTag {
    #[allow(dead_code)]
    id: String,
    kind: EntityKind,
}

/// Install the global logger that mirrors to stderr AND captures into
/// the HUD ringbuffer. Returns the receiver — passed back into
/// `run` and stored as a Bevy resource. Re-exported from the `hud`
/// module so `main.rs` doesn't have to know about either.
pub fn install_logger(default_filter: &str) -> flume::Receiver<crate::hud::log_capture::CapturedLog> {
    crate::hud::log_capture::install(default_filter)
}

pub fn run(
    config: Config,
    config_path: String,
    shared_config: std::sync::Arc<std::sync::RwLock<Config>>,
    bridge_rx: flume::Receiver<WorldUpdate>,
    headless: bool,
    conn_registry: ConnRegistry,
    match_state: controls::MatchStateLock,
    initial_slots: SpawnSlots,
    bridge_tx: flume::Sender<WorldUpdate>,
    shared_world: World,
    log_rx: flume::Receiver<crate::hud::log_capture::CapturedLog>,
) {
    let mut app = App::new();
    let team_sides = config.teams.clone();
    app.insert_resource(BridgeRx(bridge_rx))
        .insert_resource(SimEntities::default())
        .insert_resource(SimConfigRes(config))
        .insert_resource(ConfigPath(config_path))
        .insert_resource(SharedConfig(shared_config))
        .insert_resource(Headless(headless))
        .insert_resource(conn_registry)
        .insert_resource(ChordStateRes::default())
        .insert_resource(initial_slots)
        .insert_resource(TeamSidesRes(team_sides))
        .insert_resource(match_state)
        .insert_resource(HumansVisible(false))
        .insert_resource(NeopixelRegistry::default())
        .insert_resource(GroundBeamRegistry::default())
        .insert_resource(Ld06BeamRegistry::default())
        .insert_resource(Ld06BeamsVisible(false))
        .insert_resource(DebugVolumesVisible(false))
        .insert_resource(DebugVolumeRegistry::default())
        .insert_resource(crate::joints::JointAssets::default())
        .insert_resource(crate::joints::JointEntities::default())
        .insert_resource(AdversaryState::default())
        .insert_resource(UpdatesTx(bridge_tx))
        .insert_resource(SharedWorld(shared_world))
        .insert_resource(crate::hud::log_capture::LogCaptureRx(log_rx));

    if headless {
        // Run a fixed-tick loop with no rendering.
        app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(
            Duration::from_secs_f32(1.0 / 60.0),
        )));
    } else {
        // Cap the render schedule to ~30 fps. With the default
        // `Continuous` mode, idle frames go through the full pipeline
        // at vsync (60+ Hz) and burn a CPU core for nothing — 30 fps
        // is plenty to follow a simulation and halves the cost.
        use bevy::winit::{UpdateMode, WinitSettings};
        app.insert_resource(WinitSettings {
            focused_mode: UpdateMode::reactive(Duration::from_millis(33)),
            unfocused_mode: UpdateMode::reactive_low_power(Duration::from_secs(1)),
        });
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
                })
                .disable::<bevy::log::LogPlugin>(),
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
            setup_joint_assets,
            controls::spawn_initial_adversary,
        ),
    )
    .add_systems(Update, drain_bridge);
    if !headless {
        app.add_plugins(crate::hud::HudPlugin);
        app.add_systems(Startup, (log_render_adapter, install_noise_normal_map));
        app.add_systems(
            Update,
            (
                start_human_animations,
                toggle_collision_overlay,
                toggle_humans_visibility,
                apply_humans_visibility,
                ensure_mesh_normals,
                texturize_loaded_materials,
                controls::chord_input,
                controls::render_chord_overlay,
                controls::drive_adversary,
                toggle_debug_volumes_visibility,
                toggle_ld06_visibility,
            ),
        );
    }

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

fn setup_joint_assets(
    mut joints: ResMut<crate::joints::JointAssets>,
    asset_server: Option<Res<AssetServer>>,
    config: Res<SimConfigRes>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let Some(asset_server) = asset_server else { return };
    let assets_dir = format!("{}/assets", env!("CARGO_MANIFEST_DIR"));
    let load_for = |cfg: &crate::config::RobotConfig| -> std::collections::HashMap<String, Handle<Scene>> {
        let mut out = std::collections::HashMap::new();
        let Some(model) = cfg.model.as_ref() else { return out };
        for j in &model.joints {
            let key = j.file
                .strip_prefix("sim/assets/")
                .unwrap_or(&j.file);
            let file_key = key.split('#').next().unwrap_or(key);
            let full_path = format!("{}/{}", assets_dir, file_key);
            if !std::path::Path::new(&full_path).exists() {
                log::warn!("joint GLB not found: {full_path} — skipping joint {:?}", j.name);
                continue;
            }
            out.insert(j.name.clone(), asset_server.load(format!("{}#Scene0", key)));
        }
        out
    };
    joints
        .by_kind
        .insert(RobotKind::Galipeur, load_for(&config.0.galipeur));
    joints
        .by_kind
        .insert(RobotKind::Pami, load_for(&config.0.pami));
}

fn setup_robot_assets(
    mut commands: Commands,
    asset_server: Option<Res<AssetServer>>,
    config: Res<SimConfigRes>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let Some(asset_server) = asset_server else { return };

    let assets_dir = format!("{}/assets", env!("CARGO_MANIFEST_DIR"));
    let load = |cfg: &crate::config::RobotConfig| -> Option<Handle<Scene>> {
        let visual = cfg.model.as_ref()?.visual.as_deref()?;
        // Asset paths are resolved against the AssetPlugin `file_path`
        // (configured to `sim/assets/`), but our TOML stores workspace-
        // relative paths like `sim/assets/robots/galipeur.glb`. Strip the
        // `sim/assets/` prefix so the asset loader can find it.
        let key = visual
            .strip_prefix("sim/assets/")
            .unwrap_or(visual);
        // Strip the `#Scene0` fragment to check the actual file on disk.
        let file_key = key.split('#').next().unwrap_or(key);
        let full_path = format!("{}/{}", assets_dir, file_key);
        if !std::path::Path::new(&full_path).exists() {
            log::warn!("GLB not found: {full_path} — using placeholder cuboid");
            return None;
        }
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
    config: Res<SimConfigRes>,
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
    // Strat-aligned frame: X is lateral (2 * x_half_mm → Bevy Z),
    // Y is longitudinal (y_max_mm → Bevy X). Table centre on Bevy
    // sits at (y_max/2, _, 0).
    let x_span_m = (config.0.field.x_half_mm as f32 * 2.0) * MM; // lateral → bevy Z
    let y_span_m = config.0.field.y_max_mm as f32 * MM;           // longitudinal → bevy X
    let cx_world_m = y_span_m * 0.5;  // bevy X centre
    let cz_world_m = 0.0;             // bevy Z centre (symmetric)

    // Table stand (visual only, rendered from y=0 up to y=stand_h).
    if stand_h > 1e-4 {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(y_span_m, stand_h, x_span_m))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.33, 0.30),
                perceptual_roughness: 0.95,
                ..default()
            })),
            Transform::from_xyz(cx_world_m, stand_h * 0.5, cz_world_m),
        ));

        // Floor around the stand (big grey plane so humans don't look like
        // they're walking on the void).
        let floor_side = (x_span_m + y_span_m) * 3.0;
        commands.spawn((
            Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(floor_side * 0.5, floor_side * 0.5)))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.12, 0.12, 0.13),
                perceptual_roughness: 1.0,
                ..default()
            })),
            Transform::from_xyz(cx_world_m, 0.0, cz_world_m),
        ));
    }

    // Strat-axis gizmo at sim (0, 0) (Down-wall centre). RGB = XYZ strat.
    // Strat frame is right-handed: +X (right) × +Y (forward) = +Z (up).
    let axis_len_m = 1.0_f32;
    let axis_thickness_m = 0.04_f32;
    let axis_y_off = stand_h + 0.05;
    // Strat +X (red, toward Right wall) → bevy +Z.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(axis_thickness_m, axis_thickness_m, axis_len_m))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.0, 0.0),
            emissive: LinearRgba::new(2.0, 0.0, 0.0, 1.0),
            unlit: true,
            ..default()
        })),
        Transform::from_xyz(0.0, axis_y_off, axis_len_m * 0.5),
    ));
    // Strat +Y (green, toward Up wall) → bevy +X.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(axis_len_m, axis_thickness_m, axis_thickness_m))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 1.0, 0.0),
            emissive: LinearRgba::new(0.0, 2.0, 0.0, 1.0),
            unlit: true,
            ..default()
        })),
        Transform::from_xyz(axis_len_m * 0.5, axis_y_off, 0.0),
    ));
    // Strat +Z (blue, up) → bevy +Y.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(axis_thickness_m, axis_len_m, axis_thickness_m))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.3, 1.0),
            emissive: LinearRgba::new(0.0, 0.5, 2.0, 1.0),
            unlit: true,
            ..default()
        })),
        Transform::from_xyz(0.0, axis_y_off + axis_len_m * 0.5, 0.0),
    ));
    // Origin marker (white cube) at Bevy (0, 0, 0).
    let origin_side = axis_thickness_m * 2.5;
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(origin_side, origin_side, origin_side))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 1.0, 1.0),
            emissive: LinearRgba::new(2.0, 2.0, 2.0, 1.0),
            unlit: true,
            ..default()
        })),
        Transform::from_xyz(0.0, axis_y_off, 0.0),
    ));

    // Every table obstacle — including the flat ground — goes through the
    // same rendering path, offset up by the stand height.
    let mut texture_cache: std::collections::HashMap<String, Handle<Image>> =
        std::collections::HashMap::new();

    // Playmat normalization helpers: u maps strat Y (0..y_max) to texture
    // (0..1); v maps strat X (-x_half..+x_half) to texture (0..1).
    let x_half_mm = config.0.field.x_half_mm as f32;
    let y_max_mm = config.0.field.y_max_mm as f32;
    let x_span_mm = x_half_mm * 2.0;
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
        let w = ((x1 - x0) * MM).abs();  // lateral extent → bevy Z
        let d = ((y1 - y0) * MM).abs();  // longitudinal extent → bevy X
        let h = obs.height_mm * MM;
        if w < 1e-4 || d < 1e-4 {
            continue;
        }
        // strat_x (lateral) → bevy +Z.  strat_y (longitudinal) → bevy +X.
        let cx = (y0 + y1) * 0.5 * MM;
        let cz = (x0 + x1) * 0.5 * MM;

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
        // Cuboid: bevy (X=d=longitudinal, Y=h, Z=w=lateral)
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(d, visual_h, w))),
            MeshMaterial3d(materials.add(body_material)),
            Transform::from_xyz(cx, stand_h + visual_h * 0.5, cz),
        ));

        // Playmat decal on the top face. The UV transform crops the
        // region of the field-wide image corresponding to this obstacle's
        // AABB, so a playmat tile on top of `granary_ground` shows the
        // correct portion of the playing area.
        if obs.use_playmat {
            if let Some(handle) = playmat_handle.as_ref() {
                // The texture is authored in landscape (long axis = U,
                // short axis = V) with the granary near the TOP of the
                // image (v ≈ 0). The strat-aligned field has its long
                // axis along X (lateral) and short axis along Y (longitudinal).
                // Mirror across X so the image's left side (yellow
                // burrow) lands on the Left wall (strat_x < 0):
                //   texture_u = (x_half - strat_x) / x_span  (left→right across image)
                //   texture_v = (y_max - strat_y) / y_max     (granary at Up wall)
                let scale = Vec2::new(
                    /* sx */ (y1 - y0) / y_max_mm,
                    /* sy */ (x1 - x0) / x_span_mm,
                );
                let trans = Vec2::new(
                    /* tu */ (x_half_mm - x1) / x_span_mm,
                    /* tv */ (y_max_mm - y0) / y_max_mm,
                );
                let pm_material = StandardMaterial {
                    base_color: Color::WHITE,
                    base_color_texture: Some(handle.clone()),
                    perceptual_roughness: 0.9,
                    uv_transform: bevy::math::Affine2::from_scale_angle_translation(
                        scale,
                        -std::f32::consts::FRAC_PI_2,
                        trans,
                    ),
                    ..default()
                };
                // Plane half-extents: bevy X=d/2 (longitudinal), bevy Z=w/2 (lateral)
                commands.spawn((
                    Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(d * 0.5, w * 0.5)))),
                    MeshMaterial3d(materials.add(pm_material)),
                    Transform::from_xyz(cx, stand_h + visual_h + 0.001, cz),
                ));
            }
        }
    }
}

fn setup_camera_and_light(
    mut commands: Commands,
    config: Res<SimConfigRes>,
    headless: Res<Headless>,
) {
    if headless.0 {
        return;
    }
    let x_span_m = (config.0.field.x_half_mm as f32 * 2.0) * MM; // lateral
    let y_span_m = config.0.field.y_max_mm as f32 * MM;          // longitudinal
    let stand_h = stand_height_m(&config.0);
    // Strat frame: table centre is (0, y_max/2) in sim coords
    // → (y_max/2, _, 0) in Bevy.
    let cx = y_span_m / 2.0;
    let cy = 0.0;

    // Initial framing: "3/4 view" at pitch ≈ 35°, distance ≈ 1.2 × diagonal.
    // PanOrbitCamera takes over afterwards — LMB drag rotates around the
    // focus, RMB drag pans it, wheel zooms.
    let table_diag = (x_span_m * x_span_m + y_span_m * y_span_m).sqrt();
    let focus = Vec3::new(cx, stand_h + 0.1, cy);
    commands.spawn((
        Camera3d::default(),
        Tonemapping::TonyMcMapface,
        Bloom::NATURAL,
        Hdr,
        Msaa::Off,
        ScreenSpaceAmbientOcclusion {
            quality_level: ScreenSpaceAmbientOcclusionQualityLevel::Medium,
            ..default()
        },
        PanOrbitCamera {
            focus,
            radius: Some(table_diag * 1.2),
            pitch: Some(35.0_f32.to_radians()),
            // Camera behind the +X axis (on the -X / Down-wall side) looking
            // toward Up wall, so the operator sees +X axis going forward.
            yaw: Some(-90.0_f32.to_radians()),
            ..default()
        },
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: true,
            shadow_depth_bias: 0.05,
            shadow_normal_bias: 0.6,
            ..default()
        },
        Transform::default().with_rotation(
            Quat::from_rotation_y(0.6) * Quat::from_rotation_x(-1.1),
        ),
    ));

    // Knock the ambient light down so the shaded sides of objects
    // actually look shaded — Bevy's default ambient is bright enough
    // to flatten everything once the directional shadow lands.
    commands.insert_resource(bevy::light::GlobalAmbientLight {
        color: Color::srgb(0.85, 0.88, 1.0),
        brightness: 200.0,
        ..default()
    });
}

fn drain_bridge(
    mut commands: Commands,
    bridge: Res<BridgeRx>,
    mut registry: ResMut<SimEntities>,
    mut transforms: Query<(&mut Transform, &SimEntityTag)>,
    mut leds: Query<(&mut Visibility, &mut SpotLight), (With<NeopixelLed>, Without<SimEntityTag>)>,
    config: Res<SimConfigRes>,
    mut assets: AssetsParam,
    humans_visible: Res<HumansVisible>,
    mut neopixels: ResMut<NeopixelRegistry>,
    mut beams: ResMut<GroundBeamRegistry>,
    mut beam_transforms: Query<
        &mut Transform,
        (With<GroundLidarBeam>, Without<SimEntityTag>, Without<NeopixelLed>),
    >,
    mut joints: crate::joints::JointsParam,
    mut debug: DebugVolumesParam,
    mut ld06: Ld06Param,
    headless: Res<Headless>,
) {
    let stand_h = stand_height_m(&config.0);

    // Mesh/material handles are only available when DefaultPlugins provided
    // the asset + rendering plugins; in headless mode we only track the
    // Transform component and skip rendering.
    let human_assets = assets.human.as_deref();
    let robot_assets = assets.robot.as_deref();
    let mut visuals = match (headless.0, assets.meshes.take(), assets.materials.take()) {
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
                // Adversary uses a stack of cylinder children whose y-origin
                // is the chassis base — bottom-anchored just like a glb
                // scene (otherwise the parent's centre-anchor offset would
                // shove the cylinders above the table top).
                let is_adversary = matches!(kind, EntityKind::Robot(RobotKind::Adversary));
                let is_scene = is_human_scene || robot_scene.is_some() || is_adversary;
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
                // Captured by the visual `with_children` block below for
                // robots that have a glb scene; consumed after `cmd` is
                // dropped to spawn the joint hierarchy.
                let mut base_entity_capture: Option<Entity> = None;

                if let Some((ref mut meshes, ref mut materials)) = visuals {
                    let (w_m, l_m) = (width_mm * MM, length_mm * MM);
                    let color = match kind {
                        EntityKind::Robot(RobotKind::Galipeur) => Color::srgb(0.2, 0.6, 1.0),
                        EntityKind::Robot(RobotKind::Pami) => Color::srgb(1.0, 0.6, 0.2),
                        EntityKind::Robot(RobotKind::Adversary) => Color::srgb(0.95, 0.15, 0.15),
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
                            let (offset, yaw) = match rk {
                                RobotKind::Galipeur => config
                                    .0
                                    .galipeur
                                    .model
                                    .as_ref()
                                    .map(|m| (m.visual_offset_mm, m.visual_yaw_rad))
                                    .unwrap_or(([0.0; 3], 0.0)),
                                RobotKind::Pami => config
                                    .0
                                    .pami
                                    .model
                                    .as_ref()
                                    .map(|m| (m.visual_offset_mm, m.visual_yaw_rad))
                                    .unwrap_or(([0.0; 3], 0.0)),
                                // Adversary never reaches this arm
                                // (`for_kind` returns None so robot_scene
                                // is None), but the match must be
                                // exhaustive.
                                RobotKind::Adversary => ([0.0; 3], 0.0),
                            };
                            let scene_handle = robot_scene.clone().unwrap();
                            // Per-team outline colour so the two robots
                            // are distinguishable at a glance. Side is
                            // baked into the robot id ("galipeur-left"
                            // / "pami-right"); we look up the side's
                            // team colour via the TOML's
                            // `[teams]` mapping.
                            let outline_colour = team_outline_color(&id, &config.0.teams);
                            cmd.with_children(|p| {
                                let e = p
                                    .spawn((
                                        SceneRoot(scene_handle),
                                        Transform {
                                            translation: Vec3::new(
                                                offset[0] * MM,
                                                offset[1] * MM,
                                                offset[2] * MM,
                                            ),
                                            rotation: Quat::from_rotation_y(yaw),
                                            scale: Vec3::ONE,
                                        },
                                        // Per-team ink-style outline on every
                                        // mesh in the glTF scene. `AsyncScene…`
                                        // waits for the scene to load, then
                                        // sprinkles `InheritOutline` onto
                                        // every child so the inverted-hull
                                        // pass is applied mesh-by-mesh.
                                        OutlineVolume {
                                            visible: true,
                                            width: 4.0,
                                            colour: outline_colour,
                                        },
                                        AsyncSceneInheritOutline::default(),
                                    ))
                                    .id();
                                base_entity_capture = Some(e);
                            });
                        }
                        EntityKind::Robot(RobotKind::Adversary) => {
                            // Two-stage opponent: Ø450 mm base 350 mm tall,
                            // then a Ø70 mm antenna 80 mm tall on top
                            // (total 430 mm). Mirrors the collision
                            // primitives in `spawn_adversary` so what's
                            // drawn is what gets raycast.
                            let base_r = 0.225;
                            let base_h = 0.350;
                            let antenna_r = 0.035;
                            let antenna_h = 0.080;
                            let mat = materials.add(StandardMaterial {
                                base_color: color,
                                perceptual_roughness: 0.55,
                                metallic: 0.05,
                                ..default()
                            });
                            cmd.with_children(|p| {
                                p.spawn((
                                    Mesh3d(meshes.add(Cylinder::new(base_r, base_h))),
                                    MeshMaterial3d(mat.clone()),
                                    Transform::from_xyz(0.0, base_h * 0.5, 0.0),
                                    OutlineVolume {
                                        visible: true,
                                        width: 3.0,
                                        colour: Color::BLACK,
                                    },
                                ));
                                p.spawn((
                                    Mesh3d(meshes.add(Cylinder::new(antenna_r, antenna_h))),
                                    MeshMaterial3d(mat.clone()),
                                    Transform::from_xyz(0.0, base_h + antenna_h * 0.5, 0.0),
                                    OutlineVolume {
                                        visible: true,
                                        width: 3.0,
                                        colour: Color::BLACK,
                                    },
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
                            RobotKind::Adversary => None,
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
                            RobotKind::Adversary => &[],
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

                        // Ground lidars: render each beam as a
                        // translucent red cylinder of `max_range_mm`
                        // pointing along `direction` from `position_mm`.
                        // No raytracing yet — purely visual.
                        let ground_lidars: &[GroundLidar] = match rk {
                            RobotKind::Galipeur => &config.0.galipeur.ground_lidars,
                            RobotKind::Pami => &config.0.pami.ground_lidars,
                            RobotKind::Adversary => &[],
                        };
                        if !ground_lidars.is_empty() {
                            let beam_bundles = build_ground_lidar_spawns(
                                ground_lidars,
                                y_off,
                                meshes,
                                materials,
                            );
                            let mut entries: Vec<(Entity, GroundBeamGeom)> =
                                Vec::with_capacity(beam_bundles.len());
                            cmd.with_children(|parent| {
                                for (bundle, geom) in beam_bundles {
                                    let e = parent.spawn(bundle).id();
                                    entries.push((e, geom));
                                }
                            });
                            beams.0.insert(id.clone(), entries);
                        }

                        // LD06 360° beam viz: 12 hidden cyan cylinders
                        // (one per ray in a packet). Each `Ld06Hits`
                        // event rewrites their transforms in place so
                        // the slice rotates with the rotor. Galipeur
                        // only — pami doesn't carry a 360° lidar.
                        if matches!(rk, RobotKind::Galipeur) {
                            let beam_bundles = build_ld06_beams(
                                config.0.galipeur.lidar_height_mm,
                                y_off,
                                meshes,
                                materials,
                            );
                            let mut entities: Vec<Entity> = Vec::with_capacity(12);
                            cmd.with_children(|parent| {
                                for bundle in beam_bundles {
                                    entities.push(parent.spawn(bundle).id());
                                }
                            });
                            let arr: [Entity; 12] = entities
                                .try_into()
                                .expect("build_ld06_beams returns exactly 12 entities");
                            ld06.registry.0.insert(id.clone(), arr);
                        }
                    }
                }
                let entity = cmd.id();
                let robot_id_for_joints = id.clone();
                registry.0.insert(id, (entity, info));
                log::info!("[sim-app] spawned {:?}", kind);

                // Drop the borrow on `commands` so we can spawn joint
                // children below. `cmd` is no longer used after this.
                drop(cmd);

                if let (EntityKind::Robot(rk), Some(base), Some(jassets)) =
                    (kind, base_entity_capture, joints.assets.as_ref())
                {
                    let joints_slice: &[crate::config::JointSpec] = match rk {
                        RobotKind::Galipeur => config
                            .0
                            .galipeur
                            .model
                            .as_ref()
                            .map_or(&[][..], |m| m.joints.as_slice()),
                        RobotKind::Pami => config
                            .0
                            .pami
                            .model
                            .as_ref()
                            .map_or(&[][..], |m| m.joints.as_slice()),
                        RobotKind::Adversary => &[][..],
                    };
                    if !joints_slice.is_empty() {
                        let n = crate::joints::spawn_joint_tree(
                            &mut commands,
                            &robot_id_for_joints,
                            rk,
                            base,
                            joints_slice,
                            jassets,
                            &mut joints.entities,
                        );
                        log::info!(
                            "[sim-app] spawned {} joint(s) for {:?}",
                            n, rk
                        );
                    }
                }
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
                beams.0.remove(&id);
                joints.entities.remove_robot(&id);
                // Robot's children (incl. debug volumes) get despawned
                // recursively via the parent — drop the registry entry
                // so the next spawn under the same id starts fresh.
                debug.registry.0.remove(&id);
            }
            WorldUpdate::ActuatorState { id, modules } => {
                // Determine which kind this id refers to so we can
                // pull the matching `joints` config. (Spawn-time we
                // tracked the kind in `registry`.)
                let kind = registry.0.get(&id).map(|(_, info)| info.kind);
                let Some(crate::world::EntityKind::Robot(rk)) = kind else {
                    continue;
                };
                let joints_slice: &[crate::config::JointSpec] = match rk {
                    RobotKind::Galipeur => config
                        .0
                        .galipeur
                        .model
                        .as_ref()
                        .map_or(&[][..], |m| m.joints.as_slice()),
                    RobotKind::Pami => config
                        .0
                        .pami
                        .model
                        .as_ref()
                        .map_or(&[][..], |m| m.joints.as_slice()),
                    RobotKind::Adversary => &[][..],
                };
                if joints_slice.is_empty() {
                    continue;
                }
                crate::joints::apply_actuator_state(
                    &id,
                    &modules,
                    &joints.entities,
                    joints_slice,
                    &mut joints.transforms,
                );
            }
            WorldUpdate::GroundLidarHits { id, distances_mm } => {
                let Some(entries) = beams.0.get(&id) else {
                    log::debug!("ground hits for unknown robot {id}");
                    continue;
                };
                let mut applied = 0u32;
                let mut missed = 0u32;
                for (i, d_mm) in distances_mm.iter().enumerate() {
                    let Some((entity, geom)) = entries.get(i) else { break };
                    let hit_m = d_mm * MM;
                    let scale_y = if geom.max_range_m > 0.0 {
                        (hit_m / geom.max_range_m).max(1e-4)
                    } else {
                        1e-4
                    };
                    let center = geom.start_bevy + geom.dir_bevy * (hit_m * 0.5);
                    if let Ok(mut t) = beam_transforms.get_mut(*entity) {
                        t.translation = center;
                        t.scale = Vec3::new(1.0, scale_y, 1.0);
                        applied += 1;
                    } else {
                        missed += 1;
                    }
                }
                log::debug!(
                    "[bevy] GroundLidarHits {id}: applied={applied} missed={missed} len={}",
                    distances_mm.len(),
                );
            }
            WorldUpdate::Ld06Hits {
                id,
                start_angle_deg,
                end_angle_deg,
                distances_mm,
            } => {
                let Some(entities) = ld06.registry.0.get(&id) else {
                    log::debug!("ld06 hits for unknown robot {id}");
                    continue;
                };
                // Body-frame Z (up) of the LD06 emitter — used as the
                // start point of every ray. Galipeur only (the only
                // robot with a 360° lidar today).
                let h_mm = config.0.galipeur.lidar_height_mm;
                let parent_y_off = 0.0; // glb-anchored: visual scene is bottom-anchored.
                let origin = body_pos_to_bevy([0.0, 0.0, h_mm], parent_y_off);
                // 12 rays uniformly distributed on `[start, end]` —
                // matches the firmware step formula `step = (end-start)/11`.
                let span = (end_angle_deg - start_angle_deg + 360.0).rem_euclid(360.0);
                let step = if span > 0.0 { span / 11.0 } else { 0.0 };
                for i in 0..12 {
                    let entity = entities[i];
                    let local_deg = start_angle_deg + step * i as f32;
                    let a = local_deg.to_radians();
                    let dir_body = [a.cos(), a.sin(), 0.0];
                    let dir = body_dir_to_bevy(dir_body);
                    let length_m = (distances_mm[i] as f32) * MM;
                    let safe_len = length_m.max(1e-4);
                    let center = origin + dir * (length_m * 0.5);
                    let rot = Quat::from_rotation_arc(Vec3::Y, dir);
                    if let Ok(mut t) = ld06.beam_transforms.get_mut(entity) {
                        t.translation = center;
                        t.rotation = rot;
                        // Cylinder default length is 1m — scale Y to fit.
                        t.scale = Vec3::new(1.0, safe_len, 1.0);
                    }
                }
                // Visibility is owned by `toggle_ld06_visibility`; this
                // arm only updates transforms.
                let _ = ld06.visible.0;
            }
            WorldUpdate::DebugVolumes { id, volumes } => {
                let Some((parent_entity, half_h, bottom_anchored)) = registry
                    .0
                    .get(&id)
                    .map(|(e, info)| (*e, info.half_height_m, info.bottom_anchored))
                else {
                    continue;
                };
                let Some((ref mut meshes, ref mut materials)) = visuals else {
                    continue;
                };
                // Drop any previous set so the registry stays in sync
                // with what the robot just pushed (last-writer-wins).
                if let Some(old) = debug.registry.0.remove(&id) {
                    for e in old {
                        commands.entity(e).despawn();
                    }
                }
                if volumes.is_empty() {
                    continue;
                }
                // Volumes live in the robot's body frame: x_forward,
                // y_left, z_up. Bevy uses x, y_up, z. Reuse the same
                // axis swap as the neopixel/lidar overlay code.
                let y_off = if bottom_anchored { 0.0 } else { -half_h };
                let initial_vis = if debug.visible.0 {
                    // Force Visible (not Inherited) so the overlay
                    // stays visible regardless of the parent robot's
                    // visibility chain (LOD culling, hide-all toggles,
                    // etc.).
                    Visibility::Visible
                } else {
                    Visibility::Hidden
                };
                let mut spawned = Vec::with_capacity(volumes.len());
                // Collect per-frame bundles: body-frame volumes get
                // parented to the robot, table-frame ones are root entities.
                let mut body_bundles: Vec<DebugVolBundle> = Vec::new();
                let mut table_bundles: Vec<DebugVolBundle> = Vec::new();
                for vol in &volumes {
                    let bundle = debug_vol_bundle(vol, meshes, materials);
                    match vol.frame() {
                        DebugVolumeFrame::Body => {
                            let mut b = bundle;
                            b.transform = debug_vol_transform(vol, y_off);
                            body_bundles.push(b);
                        }
                        DebugVolumeFrame::Table => {
                            let mut b = bundle;
                            b.transform = debug_vol_transform(vol, stand_h);
                            table_bundles.push(b);
                        }
                    }
                }
                if !body_bundles.is_empty() {
                    commands.entity(parent_entity).with_children(|parent| {
                        for b in body_bundles {
                            let e = parent
                                .spawn((
                                    b.mesh, b.material, b.transform,
                                    initial_vis, DebugVolumeOverlay,
                                ))
                                .id();
                            spawned.push(e);
                        }
                    });
                }
                for b in table_bundles {
                    let e = commands
                        .spawn((
                            b.mesh, b.material, b.transform,
                            initial_vis, DebugVolumeOverlay,
                        ))
                        .id();
                    spawned.push(e);
                }
                debug.registry.0.insert(id, spawned);
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

/// Eurobot team palette → outline colour. Robot ids are formatted
/// as `{kind}-{side}` by `controls::launch_robot`, so we parse the
/// `-left` / `-right` suffix and consult the live `[teams]` map to
/// pick the team's signature colour. Falls back to black for ids
/// that don't carry a side (the manual adversary, humans).
fn team_outline_color(robot_id: &str, teams: &crate::config::TeamSides) -> Color {
    // Eurobot signature colours, slightly toned down for readability
    // against the wood / playmat backdrop.
    const YELLOW: Color = Color::srgb(1.0, 0.78, 0.0);
    const BLUE: Color = Color::srgb(0.12, 0.37, 0.74);
    let side = if robot_id.ends_with("-left") {
        crate::config::Side::Left
    } else if robot_id.ends_with("-right") {
        crate::config::Side::Right
    } else {
        return Color::BLACK;
    };
    match teams.team_of(side) {
        Some("blue") => BLUE,
        Some("yellow") => YELLOW,
        _ => Color::BLACK,
    }
}

fn body_pos_to_bevy(pos_mm: [f32; 3], parent_y_off_m: f32) -> Vec3 {
    // Strat frame is (x_right, y_forward, z_up), right-handed.
    // Bevy world is right-handed Y-up. Mapping:
    //   strat +X (right)   → bevy +Z
    //   strat +Y (forward) → bevy +X
    //   strat +Z (up)      → bevy +Y
    Vec3::new(
        pos_mm[1] * MM,
        parent_y_off_m + pos_mm[2] * MM,
        pos_mm[0] * MM,
    )
}

/// Convert a body-frame unit direction `(x_right, y_fwd, z_up)` into
/// the same Bevy axis mapping used for positions.
fn body_dir_to_bevy(dir: [f32; 3]) -> Vec3 {
    Vec3::new(dir[1], dir[2], dir[0]).normalize_or_zero()
}

/// Intermediate representation for a debug volume ready to spawn.
struct DebugVolBundle {
    mesh: Mesh3d,
    material: MeshMaterial3d<StandardMaterial>,
    transform: Transform,
}

/// Build the mesh + material for a debug volume (transform filled later).
fn debug_vol_bundle(
    vol: &DebugVolume,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> DebugVolBundle {
    match vol {
        DebugVolume::Box { half_size_mm, rgba, .. } => {
            // half_size_mm is [strat_x, strat_y, strat_z].
            // Mapping: strat +X → bevy +Z, strat +Y → bevy +X, strat +Z → bevy +Y.
            let size_m = Vec3::new(
                half_size_mm[1] * 2.0 * MM,
                half_size_mm[2] * 2.0 * MM,
                half_size_mm[0] * 2.0 * MM,
            );
            DebugVolBundle {
                mesh: Mesh3d(meshes.add(Cuboid::new(size_m.x, size_m.y, size_m.z))),
                material: MeshMaterial3d(materials.add(debug_volume_material(*rgba))),
                transform: Transform::IDENTITY,
            }
        }
        DebugVolume::Cylinder { radius_mm, height_mm, rgba, .. } => {
            let r = *radius_mm * MM;
            let h = *height_mm * MM;
            DebugVolBundle {
                mesh: Mesh3d(meshes.add(Cylinder::new(r, h))),
                material: MeshMaterial3d(materials.add(debug_volume_material(*rgba))),
                transform: Transform::IDENTITY,
            }
        }
        DebugVolume::Text { size_mm, rgba, .. } => {
            // Render text as a small sphere placeholder — full billboard
            // text requires the 2D text pipeline which cannot be mixed
            // into the 3D overlay system trivially.
            let r = *size_mm * 0.5 * MM;
            DebugVolBundle {
                mesh: Mesh3d(meshes.add(Sphere::new(r))),
                material: MeshMaterial3d(materials.add(debug_volume_material(*rgba))),
                transform: Transform::IDENTITY,
            }
        }
    }
}

/// Compute the Bevy transform for a debug volume in its reference frame.
/// `y_off` is the parent-anchor offset for body-frame, or `stand_h` for
/// table-frame.
fn debug_vol_transform(vol: &DebugVolume, y_off: f32) -> Transform {
    match vol {
        DebugVolume::Box { center_mm, yaw_rad, .. } => {
            body_transform_to_bevy(*center_mm, *yaw_rad, y_off)
        }
        DebugVolume::Cylinder { center_mm, height_mm, .. } => {
            let mut t = body_transform_to_bevy(*center_mm, 0.0, y_off);
            t.translation.y += *height_mm * MM * 0.5;
            t
        }
        DebugVolume::Text { position_mm, .. } => {
            body_transform_to_bevy(*position_mm, 0.0, y_off)
        }
    }
}

/// Body-frame (x_right, y_forward, z_up) pose → Bevy local transform.
/// Single source of truth for the sim↔Bevy convention used across
/// every body-relative entity (robot pose, collision overlays, lidar
/// beams, `WorldUpdate::DebugVolumes`, etc).
///
/// The strat→Bevy mapping is: strat +X → bevy +Z, strat +Y → bevy +X,
/// strat +Z → bevy +Y (proper rotation, det = 1). A CCW yaw around
/// strat +Z is a CCW rotation around bevy +Y — no sign flip needed.
///
/// `y_off_m` is the parent-anchor correction: 0 for bottom-anchored
/// scenes (glb robots, adversary cylinder stack) and `-half_h` for
/// centre-anchored cuboid robots.
fn body_transform_to_bevy(pos_mm: [f32; 3], yaw_rad: f32, y_off_m: f32) -> Transform {
    Transform {
        translation: body_pos_to_bevy(pos_mm, y_off_m),
        rotation: Quat::from_rotation_y(yaw_rad),
        scale: Vec3::ONE,
    }
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
    let push = |pos_body: [f32; 3],
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

type GroundLidarSpawn = (
    Mesh3d,
    MeshMaterial3d<StandardMaterial>,
    Transform,
    GroundLidarBeam,
);

fn build_ground_lidar_spawns(
    lidars: &[GroundLidar],
    parent_y_offset_m: f32,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Vec<(GroundLidarSpawn, GroundBeamGeom)> {
    // One shared material (translucent red, unlit) across all beams.
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.15, 0.15, 0.45),
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        unlit: true,
        ..default()
    });

    let mut out = Vec::with_capacity(lidars.len());
    for gl in lidars {
        if !gl.enabled {
            continue;
        }
        let length_m = gl.max_range_mm * MM;
        let radius_m = gl.beam_radius_mm * MM;
        if length_m <= 0.0 {
            continue;
        }
        let start = body_pos_to_bevy(gl.position_mm, parent_y_offset_m);
        // Beam lies in the body XY plane (parallel to the ground):
        // direction = (cos θ, sin θ, 0) in body frame.
        let dir_body = [gl.theta_rad.cos(), gl.theta_rad.sin(), 0.0];
        let dir = body_dir_to_bevy(dir_body);
        // Bevy's `Cylinder` has its axis along +Y, centred at its
        // midpoint. Rotate so +Y aligns with `dir`, then offset the
        // centre so the emitter end sits at `start` and the beam
        // extends outward by `length_m`.
        let rot = Quat::from_rotation_arc(Vec3::Y, dir);
        let center = start + dir * (length_m * 0.5);
        let geom = GroundBeamGeom {
            start_bevy: start,
            dir_bevy: dir,
            max_range_m: length_m,
        };
        let bundle = (
            Mesh3d(meshes.add(Cylinder::new(radius_m, length_m))),
            MeshMaterial3d(mat.clone()),
            Transform {
                translation: center,
                rotation: rot,
                ..default()
            },
            GroundLidarBeam,
        );
        out.push((bundle, geom));
    }
    out
}

type Ld06BeamSpawn = (
    Mesh3d,
    MeshMaterial3d<StandardMaterial>,
    Transform,
    Visibility,
    Ld06Beam,
);

/// Spawn 12 hidden cyan cylinders representing one LD06 packet's worth
/// of rays. The transforms are placeholders (zero-length cylinders at
/// the lidar mount point); `WorldUpdate::Ld06Hits` rewrites them in
/// place at each arriving packet so the slice "rotates" as the rotor
/// advances.
fn build_ld06_beams(
    lidar_height_mm: f32,
    parent_y_offset_m: f32,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> [Ld06BeamSpawn; 12] {
    // One shared cyan material for the whole slice. Distinct from the
    // ground-lidar red so the two viz layers are easy to tell apart.
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.2, 0.8, 1.0, 0.55),
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        unlit: true,
        ..default()
    });
    // Thin cylinder, length 1m by default. A non-uniform `scale.y`
    // applied at update time fits the cylinder to the actual hit
    // distance (Bevy's `Cylinder` has its axis along +Y).
    let mesh = meshes.add(Cylinder::new(0.003, 1.0));
    let placeholder = body_pos_to_bevy([0.0, 0.0, lidar_height_mm], parent_y_offset_m);
    std::array::from_fn(|_| {
        (
            Mesh3d(mesh.clone()),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(placeholder),
            Visibility::Hidden,
            Ld06Beam,
        )
    })
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
                        -center_mm[1] * MM,
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
/// Primitive points `[x_mm, y_mm]` are body-frame (+X forward, +Y left). They
/// map to Bevy local `(x*MM, 0, -y*MM)` and `(x*MM, height_m, -y*MM)` — the
/// proper -90° X rotation also used by `body_pos_to_bevy`. Winding assumes
/// the input polygon is CCW in the body frame; flipping the y component to
/// -bevy_z inverts handedness in (bevy_x, bevy_z), so we reverse the winding
/// order here so faces still look "outward".
fn build_polygon_prism(points_mm: &[[f32; 2]], height_m: f32) -> Mesh {
    let n = points_mm.len();
    assert!(n >= 3, "polygon must have >= 3 points");

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(2 * n);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(2 * n);
    let mut indices: Vec<u32> = Vec::new();

    for &[x, y] in points_mm {
        positions.push([x * MM, 0.0, -y * MM]);
        normals.push([0.0, -1.0, 0.0]);
    }
    for &[x, y] in points_mm {
        positions.push([x * MM, height_m, -y * MM]);
        normals.push([0.0, 1.0, 0.0]);
    }

    let nu = n as u32;
    // After the y → -z flip, what was a CCW body polygon is CW in the bevy
    // (X, Z) plane. The OLD windings were tuned for CCW; reverse each
    // triangle so faces still point outward.
    for i in 1..nu - 1 {
        indices.extend_from_slice(&[0, i, i + 1]);
    }
    for i in 1..nu - 1 {
        indices.extend_from_slice(&[nu, nu + i + 1, nu + i]);
    }
    for i in 0..nu {
        let next = (i + 1) % nu;
        indices.extend_from_slice(&[i, nu + next, next]);
        indices.extend_from_slice(&[i, nu + i, nu + next]);
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

/// Print the wgpu adapter Bevy ended up with, and warn loudly if it
/// fell back to a software renderer (llvmpipe). The latter pegs the
/// CPU at ~150% with very low FPS, so the user knows to install GPU
/// drivers / set `WGPU_BACKEND=vulkan|gl` before debugging further.
fn log_render_adapter(adapter: Option<Res<RenderAdapterInfo>>) {
    let Some(adapter) = adapter else { return };
    let info = &adapter.0;
    log::info!(
        "render adapter: {} [{:?}, {:?}], driver: {} {}",
        info.name, info.device_type, info.backend, info.driver, info.driver_info,
    );
    let is_software = format!("{:?}", info.device_type) == "Cpu"
        || info.name.to_ascii_lowercase().contains("llvmpipe")
        || info.name.to_ascii_lowercase().contains("software");
    if is_software {
        log::warn!(
            "wgpu fell back to a software renderer — expect ~150% CPU and \
             very low FPS. Set WGPU_FORCE_FALLBACK_ADAPTER=0 and ensure a \
             GPU driver is installed (Vulkan: vulkan-tools + mesa-vulkan-drivers; \
             GL: try WGPU_BACKEND=gl)."
        );
    }
}

/// Procedural normal map: a small tileable noise texture applied to
/// every freshly loaded `StandardMaterial` that doesn't already have a
/// normal map. Adds surface micro-variation to the otherwise flat-
/// shaded glb parts so they don't look like uniform plastic.
#[derive(Resource, Clone)]
struct NoiseNormalMap(Handle<Image>);

fn install_noise_normal_map(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    use bevy::image::Image;
    use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

    const SIDE: u32 = 256;
    let mut rng = fastrand::Rng::with_seed(0x42);
    let mut data = Vec::with_capacity((SIDE * SIDE * 4) as usize);
    // Two octaves of value-noise approximated cheaply: per-pixel
    // gradient in a small range around the +Z normal direction. Strong
    // enough to break specular highlights, gentle enough not to look
    // bumpy.
    for _ in 0..(SIDE * SIDE) {
        let dx = rng.f32() * 2.0 - 1.0;
        let dy = rng.f32() * 2.0 - 1.0;
        // Tangent-space normal: (x, y, z) where z is dominant. Encode
        // into [0, 255] with the usual normal-map convention.
        let amp = 0.08;
        let nx = dx * amp;
        let ny = dy * amp;
        let nz = (1.0 - nx * nx - ny * ny).max(0.0).sqrt();
        data.push(((nx * 0.5 + 0.5) * 255.0) as u8);
        data.push(((ny * 0.5 + 0.5) * 255.0) as u8);
        data.push(((nz * 0.5 + 0.5) * 255.0) as u8);
        data.push(255);
    }
    let img = Image::new_fill(
        Extent3d { width: SIDE, height: SIDE, depth_or_array_layers: 1 },
        TextureDimension::D2,
        &data,
        TextureFormat::Rgba8Unorm,
        bevy::asset::RenderAssetUsages::RENDER_WORLD,
    );
    let handle = images.add(img);
    commands.insert_resource(NoiseNormalMap(handle));
}

/// Apply the procedural normal map to any `StandardMaterial` that
/// doesn't already carry one. Reacts only to `Added` events to avoid
/// the get_mut → Modified feedback loop.
fn texturize_loaded_materials(
    mut materials: Option<ResMut<Assets<StandardMaterial>>>,
    mut events: MessageReader<AssetEvent<StandardMaterial>>,
    noise: Option<Res<NoiseNormalMap>>,
) {
    let Some(materials) = materials.as_mut() else { return };
    let Some(noise) = noise else { return };
    for event in events.read() {
        let AssetEvent::Added { id } = event else { continue };
        let needs_apply = matches!(materials.get(*id), Some(m)
            if !m.unlit && m.normal_map_texture.is_none());
        if !needs_apply {
            continue;
        }
        let Some(m) = materials.get_mut(*id) else { continue };
        m.normal_map_texture = Some(noise.0.clone());
        // Bump the roughness floor a little — really shiny PBR
        // surfaces look unnatural on the chassis parts.
        m.perceptual_roughness = m.perceptual_roughness.max(0.55);
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
    world: Res<SharedWorld>,
) {
    let Some(keys) = keys else { return };
    if keys.just_pressed(KeyCode::KeyC) {
        visible.0 = !visible.0;
        // Mirror the toggle into the shared `World` so the IPC raycast
        // (lidar, VLX) skips humans that are no longer rendered.
        world.0.set_humans_visible(visible.0);
    }
}

/// `L`: toggle the LD06 360° beam viz (12 cyan cylinders that follow
/// the rotor's current packet slice). Hidden by default.
fn toggle_ld06_visibility(
    keys: Option<Res<ButtonInput<KeyCode>>>,
    mut visible: ResMut<Ld06BeamsVisible>,
    mut q: Query<&mut Visibility, With<Ld06Beam>>,
) {
    let Some(keys) = keys else { return };
    if !keys.just_pressed(KeyCode::KeyL) {
        return;
    }
    visible.0 = !visible.0;
    let target = if visible.0 { Visibility::Inherited } else { Visibility::Hidden };
    for mut v in &mut q {
        *v = target;
    }
}

/// `V`: toggle every translucent debug volume the robots have pushed.
/// Hidden by default so a fresh sim doesn't show the rectangles
/// unless the operator explicitly asks for them.
fn toggle_debug_volumes_visibility(
    keys: Option<Res<ButtonInput<KeyCode>>>,
    mut visible: ResMut<DebugVolumesVisible>,
    mut q: Query<&mut Visibility, With<DebugVolumeOverlay>>,
) {
    let Some(keys) = keys else { return };
    if !keys.just_pressed(KeyCode::KeyV) {
        return;
    }
    visible.0 = !visible.0;
    let target = if visible.0 {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    for mut v in &mut q {
        *v = target;
    }
}

/// Build the translucent material used for every debug volume — same
/// recipe across boxes / cylinders so they read consistently when
/// stacked under H/V toggles.
fn debug_volume_material(rgba: [f32; 4]) -> StandardMaterial {
    StandardMaterial {
        base_color: Color::srgba(rgba[0], rgba[1], rgba[2], rgba[3]),
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        unlit: true,
        ..default()
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

    // CesiumMan's glTF has its local "forward" along +Z rather than +X,
    // so humans need a +π/2 pre-rotation to align with the protocol
    // (theta=0 means facing world +X). Robots' glTF is already +X-fwd
    // (onshape_fetch handles axis conversion on export), so no extra
    // heading offset there — otherwise the visual rotation wouldn't
    // match the physics and ray casts would land in the wrong
    // direction.
    let heading_offset = match info.kind {
        EntityKind::Human => -std::f32::consts::FRAC_PI_2,
        _ => 0.0,
    };
    // World pose is the body frame at (x_mm, y_mm, 0) with yaw =
    // theta_rad in body convention. Funnel through the same helper
    // every body-frame child uses so a single bug fix in axes / sign
    // covers parents AND children.
    body_transform_to_bevy(
        [pose.x_mm, pose.y_mm, 0.0],
        pose.theta_rad - heading_offset,
        y_base + y_offset,
    )
}
