//! Keyboard controls: chord-based shortcuts with an on-screen hint
//! overlay driven by a small state machine.
//!
//! Keys are matched on the *logical* keyboard layout (Bevy's `Key`),
//! not the QWERTY-positional `KeyCode`. So an AZERTY user pressing the
//! key labelled `Q` triggers the same action as a QWERTY user pressing
//! `Q`. Without this, the chord would fire on whatever key sits at the
//! QWERTY-Q position regardless of what's printed on the keycap.
//!
//! - `K`                kill-all: shutdown every active IPC connection.
//! - `R`                reset: kill-all + respawn the same robots, back
//!                      to Pregame state.
//! - `T`                toggle starter: Pregame → StarterInserted →
//!                      Running. Broadcasts the new state to every
//!                      connected robot's mock starter pin.
//! - `N` `G`/`P`/`A`    spawn a robot (`cargo rgalipeur` / `cargo rpami`)
//!     `L`/`R`           on the Left or Right side of the table, or
//!                       spawn the manual `A`dversary (no side picker
//!                       — driven from ZQSD/gamepad). Refused after the
//!                       starter has been inserted — use `K` or `R`
//!                       first. The chord trigger used to be `S`, but
//!                       `S` clashes with the ZQSD reverse key once an
//!                       adversary is on the table.
//! - `X`                quit the sim. (Used to be `Q`, but `Q` is now
//!                      part of the ZQSD adversary control set.)
//! - `Esc`              cancel an in-progress chord.
//!
//! The active chord state drives a centred hint overlay so the user sees
//! which keys are available at each step.

use std::net::Shutdown;
use std::os::unix::net::UnixStream;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};

use bevy::input::keyboard::Key;
use bevy::prelude::*;
use sim_protocol::{send_msg, Pose2D, RobotKind, Side, SimMsgS2C};

use crate::bridge::WorldUpdate;
use crate::config::{CollisionPrimitive, TeamSides};
use crate::world::{EntityKind, EntitySnapshot, World};

/// State of the match in the sim. Drives both the Hello-refusal in the
/// server (no new robots once the starter is in) and the T-key
/// transitions in the chord handler.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum MatchState {
    #[default]
    Pregame,
    StarterInserted,
    Running,
}

/// Process-wide handle so the IPC listener (a non-Bevy thread) and the
/// chord handler (a Bevy system) can both consult / mutate the match
/// state under a single `Mutex`.
#[derive(Resource, Clone, Default)]
pub struct MatchStateLock(pub Arc<Mutex<MatchState>>);

impl MatchStateLock {
    pub fn get(&self) -> MatchState {
        *self.0.lock().unwrap()
    }
    pub fn set(&self, s: MatchState) {
        *self.0.lock().unwrap() = s;
    }
}

/// Shared registry of active IPC connections. `server::listen_forever`
/// pushes a cloned stream handle on accept and removes it on disconnect.
/// `kill_all` drains the list and shuts down every stream, which makes
/// each per-connection thread's blocking `recv_msg` return an error and
/// exit cleanly.
#[derive(Resource, Clone, Default)]
pub struct ConnRegistry(pub Arc<Mutex<Vec<(String, UnixStream)>>>);

impl ConnRegistry {
    pub fn register(&self, id: String, stream: UnixStream) {
        self.0.lock().unwrap().push((id, stream));
    }

    pub fn unregister(&self, id: &str) {
        self.0.lock().unwrap().retain(|(rid, _)| rid != id);
    }

    pub fn kill_all(&self) -> usize {
        let mut v = self.0.lock().unwrap();
        let n = v.len();
        for (id, s) in v.drain(..) {
            // Ask the robot to commit suicide first — gives it a clean
            // log line and avoids the mock drivers spamming channel-
            // disconnect errors before the process figures out it should
            // exit. Shutting down the stream afterwards is a belt-and-
            // suspenders: if the robot didn't receive the message it
            // still hits the `connection lost` path.
            let _ = send_msg(
                &s,
                &SimMsgS2C::Shutdown {
                    reason: format!("sim kill-all (robot_id={id})"),
                },
            );
            let _ = s.shutdown(Shutdown::Both);
        }
        n
    }

    /// Send `msg` to every currently-connected robot. Errors are
    /// logged and ignored — a dead socket is the listener thread's
    /// problem to clean up.
    pub fn broadcast(&self, msg: &SimMsgS2C) {
        let v = self.0.lock().unwrap();
        for (id, s) in v.iter() {
            if let Err(e) = send_msg(s, msg) {
                log::warn!("broadcast to {id} failed: {e}");
            }
        }
    }
}

#[derive(Default, Clone, Debug, PartialEq)]
pub enum ChordState {
    #[default]
    Idle,
    AwaitingSpawnKind,
    AwaitingSpawnSide(RobotKind),
    /// Transient message shown on the chord overlay, reverts to `Idle`
    /// after `ttl_s` seconds. Used to explain refusals like "both
    /// sides already spawned" without freezing the input.
    Flash {
        message: String,
        ttl_s: f32,
    },
}

#[derive(Resource, Default)]
pub struct ChordStateRes(pub ChordState);

/// Colour → side mapping copied from the TOML config at startup. Kept
/// separate from `SimConfig` so the chord can access it without
/// reaching into the full config resource.
#[derive(Resource, Clone)]
pub struct TeamSidesRes(pub TeamSides);

/// Which `(kind, side)` combinations the sim has explicitly launched.
/// Indexed `[Side::Left = 0, Side::Right = 1]`. `kill_all` clears every
/// flag; there is no tracking of externally-spawned robots (those
/// connecting via `cargo run` from a terminal), which is acceptable
/// for the chord UX — the user is already in charge of what they
/// started themselves.
#[derive(Resource, Default, Clone)]
pub struct SpawnSlots {
    pub galipeur: [bool; 2],
    pub pami: [bool; 2],
    /// Adversary has no side concept — single slot. Spawned and
    /// despawned in-process; never goes through `cargo run`.
    pub adversary: bool,
    /// Set by `--spawn adversary` so the Bevy startup loop knows to
    /// spawn the in-process adversary entity. Consumed and cleared by
    /// `spawn_initial_adversary` once the app is up.
    pub request_adversary_spawn: bool,
}

impl SpawnSlots {
    fn slots(&self, kind: RobotKind) -> [bool; 2] {
        match kind {
            RobotKind::Galipeur => self.galipeur,
            RobotKind::Pami => self.pami,
            // Adversary has no side dimension; pretend "both sides
            // taken" once spawned so `pick_kind` returns a flash
            // refusal instead of asking for L/R.
            RobotKind::Adversary => [self.adversary, self.adversary],
        }
    }

    pub fn set(&mut self, kind: RobotKind, side: Side, val: bool) {
        let arr = match kind {
            RobotKind::Galipeur => &mut self.galipeur,
            RobotKind::Pami => &mut self.pami,
            RobotKind::Adversary => {
                self.adversary = val;
                return;
            }
        };
        arr[side_idx(side)] = val;
    }

    pub fn clear_all(&mut self) {
        self.galipeur = [false; 2];
        self.pami = [false; 2];
        self.adversary = false;
    }

    /// List the `(kind, side)` pairs currently flagged as spawned.
    /// Used by the `R` reset to relaunch the same configuration.
    /// Adversary is intentionally absent here (no side dimension);
    /// `reset_match` reads `slots.adversary` directly and re-spawns
    /// the in-process adversary alongside the launched children.
    pub fn entries(&self) -> Vec<(RobotKind, Side)> {
        let mut out = Vec::new();
        for (i, &on) in self.galipeur.iter().enumerate() {
            if on {
                out.push((RobotKind::Galipeur, side_from_idx(i)));
            }
        }
        for (i, &on) in self.pami.iter().enumerate() {
            if on {
                out.push((RobotKind::Pami, side_from_idx(i)));
            }
        }
        out
    }
}

fn side_idx(side: Side) -> usize {
    match side {
        Side::Left => 0,
        Side::Right => 1,
    }
}

fn side_from_idx(i: usize) -> Side {
    if i == 0 { Side::Left } else { Side::Right }
}

fn side_name(side: Side) -> &'static str {
    match side {
        Side::Left => "left",
        Side::Right => "right",
    }
}

/// Manually-driven adversary robot. The sim owns its pose: ZQSD /
/// gamepad updates feed straight into `pose`, and `drive_adversary`
/// pushes the result into the world + bridge each frame.
#[derive(Resource, Default)]
pub struct AdversaryState {
    pub id: Option<String>,
    pub pose: Pose2D,
}

/// Bridge sender exposed as a Bevy resource so the chord handler and
/// adversary driver can publish `WorldUpdate`s straight into the same
/// channel the IPC server uses.
#[derive(Resource, Clone)]
pub struct UpdatesTx(pub flume::Sender<WorldUpdate>);

/// World handle exposed as a Bevy resource so the adversary's pose
/// stays in sync with the world the IPC raycasts read from.
#[derive(Resource, Clone)]
pub struct SharedWorld(pub World);

#[derive(Component)]
pub struct ChordOverlay;

/// True when an alphabetic key is pressed on the user's layout, regardless
/// of whether the corresponding QWERTY position is `KeyCode::KeyX`. Bevy's
/// `Key::Character` carries the layout-translated character, so AZERTY
/// users get the key labelled on their cap.
fn just_pressed_char(keys: &ButtonInput<Key>, want: &str) -> bool {
    keys.get_just_pressed().any(|k| match k {
        Key::Character(s) => s.eq_ignore_ascii_case(want),
        _ => false,
    })
}

fn pressed_char(keys: &ButtonInput<Key>, want: &str) -> bool {
    keys.get_pressed().any(|k| match k {
        Key::Character(s) => s.eq_ignore_ascii_case(want),
        _ => false,
    })
}

pub fn chord_input(
    keys: Option<Res<ButtonInput<Key>>>,
    time: Res<Time>,
    mut state: ResMut<ChordStateRes>,
    mut slots: ResMut<SpawnSlots>,
    conns: Res<ConnRegistry>,
    teams: Res<TeamSidesRes>,
    match_state: Res<MatchStateLock>,
    mut adversary: ResMut<AdversaryState>,
    updates: Res<UpdatesTx>,
    world: Res<SharedWorld>,
    mut config: ResMut<crate::app::SimConfigRes>,
    config_path: Res<crate::app::ConfigPath>,
    shared_config: Res<crate::app::SharedConfig>,
    keycodes: Option<Res<ButtonInput<KeyCode>>>,
    hud_focus: Option<Res<crate::hud::LogsFilterFocus>>,
) {
    // Countdown on any active Flash state — reverts to Idle when expired.
    if let ChordState::Flash { ttl_s, .. } = &mut state.0 {
        *ttl_s -= time.delta_secs();
        if *ttl_s <= 0.0 {
            state.0 = ChordState::Idle;
        }
    }

    // Block all key chords while the HUD logs-filter input is focused
    // — otherwise typing "x" / "k" / "t" would trigger kill-all / etc.
    if hud_focus.map_or(false, |f| f.0) {
        return;
    }

    let Some(keys) = keys else { return };

    // Esc still uses KeyCode (it's a function key, not a character).
    if let Some(kc) = keycodes.as_ref() {
        if kc.just_pressed(KeyCode::Escape) {
            if state.0 != ChordState::Idle {
                state.0 = ChordState::Idle;
            }
            return;
        }
    }

    match state.0.clone() {
        ChordState::Idle => {
            if just_pressed_char(&keys, "k") {
                let n = conns.kill_all();
                slots.clear_all();
                match_state.set(MatchState::Pregame);
                despawn_adversary(&mut adversary, &updates, &world);
                log::info!("kill-all: disconnected {n} robot(s)");
            } else if just_pressed_char(&keys, "r") {
                // Hot-reload toml so tweaks (e.g. visual_yaw_rad,
                // start poses) take effect without a sim restart. On
                // parse error, keep the in-memory config and surface
                // the error in the chord overlay.
                match crate::config::Config::load(&config_path.0) {
                    Ok(new_cfg) => {
                        // Update both the Bevy-side read snapshot and
                        // the shared lock the listener thread reads at
                        // handshake time, so robots launched right after
                        // `R` see the new config.
                        *shared_config.0.write().unwrap() = new_cfg.clone();
                        config.0 = new_cfg;
                        log::info!("[reset] reloaded {}", config_path.0);
                    }
                    Err(e) => {
                        log::error!("[reset] reload {} failed: {e}", config_path.0);
                        state.0 = ChordState::Flash {
                            message: format!("reload failed: {e}"),
                            ttl_s: 3.0,
                        };
                    }
                }
                reset_match(
                    &mut state,
                    &conns,
                    &mut slots,
                    &teams.0,
                    &match_state,
                    &mut adversary,
                    &updates,
                    &world,
                    &config.0,
                );
            } else if just_pressed_char(&keys, "t") {
                advance_starter(&conns, &match_state, &mut state);
            } else if just_pressed_char(&keys, "x") {
                // Also kill the robots on the way out so they don't
                // spam errors on the now-closed socket.
                let _ = conns.kill_all();
                log::info!("quit requested (X)");
                std::process::exit(0);
            } else if just_pressed_char(&keys, "n") {
                if match_state.get() != MatchState::Pregame {
                    state.0 = ChordState::Flash {
                        message: "spawn refused: starter is already in (K or R first)"
                            .into(),
                        ttl_s: 2.0,
                    };
                } else {
                    state.0 = ChordState::AwaitingSpawnKind;
                }
            }
        }
        ChordState::AwaitingSpawnKind => {
            if just_pressed_char(&keys, "g") {
                pick_kind(RobotKind::Galipeur, &mut state, &mut slots, &teams.0);
            } else if just_pressed_char(&keys, "p") {
                pick_kind(RobotKind::Pami, &mut state, &mut slots, &teams.0);
            } else if just_pressed_char(&keys, "a") {
                spawn_adversary(
                    &mut state,
                    &mut slots,
                    &mut adversary,
                    &updates,
                    &world,
                    &config.0,
                );
            }
        }
        ChordState::AwaitingSpawnSide(kind) => {
            if just_pressed_char(&keys, "l") {
                spawn_side(kind, Side::Left, &mut state, &mut slots, &teams.0);
            } else if just_pressed_char(&keys, "r") {
                spawn_side(kind, Side::Right, &mut state, &mut slots, &teams.0);
            }
        }
        ChordState::Flash { .. } => {
            // Any key (other than Esc handled above) dismisses early.
            if keys.get_just_pressed().next().is_some() {
                state.0 = ChordState::Idle;
            }
        }
    }
}

/// Toggle the simulated starter cable. Pregame → Inserted → Running →
/// no-op (a `R` reset is required to re-arm).
fn advance_starter(
    conns: &ConnRegistry,
    match_state: &MatchStateLock,
    chord: &mut ChordStateRes,
) {
    match match_state.get() {
        MatchState::Pregame => {
            conns.broadcast(&SimMsgS2C::Starter { inserted: true });
            match_state.set(MatchState::StarterInserted);
            log::info!("[match] starter inserted");
        }
        MatchState::StarterInserted => {
            conns.broadcast(&SimMsgS2C::Starter { inserted: false });
            match_state.set(MatchState::Running);
            log::info!("[match] starter removed — match running");
        }
        MatchState::Running => {
            chord.0 = ChordState::Flash {
                message: "match already running — press R to reset".into(),
                ttl_s: 2.0,
            };
        }
    }
}

/// Hard-reset: kill every robot, then relaunch the same `(kind, team)`
/// set. Match state goes back to Pregame so the strat can run a fresh
/// startup cycle.
fn reset_match(
    state: &mut ChordStateRes,
    conns: &ConnRegistry,
    slots: &mut SpawnSlots,
    teams: &TeamSides,
    match_state: &MatchStateLock,
    adversary: &mut AdversaryState,
    updates: &UpdatesTx,
    world: &SharedWorld,
    config: &crate::config::Config,
) {
    let entries = slots.entries();
    let had_adversary = slots.adversary;
    let n_killed = conns.kill_all();
    slots.clear_all();
    match_state.set(MatchState::Pregame);
    despawn_adversary(adversary, updates, world);
    log::info!(
        "[match] reset: killed {n_killed} robot(s), relaunching {} spawn(s){}",
        entries.len(),
        if had_adversary { " + adversary" } else { "" }
    );
    for (kind, side) in entries {
        launch_robot(kind, side, teams);
        slots.set(kind, side, true);
    }
    if had_adversary {
        spawn_adversary(state, slots, adversary, updates, world, config);
    }
}

fn pick_kind(
    kind: RobotKind,
    state: &mut ChordStateRes,
    slots: &mut SpawnSlots,
    teams: &TeamSides,
) {
    let [left, right] = slots.slots(kind);
    match (left, right) {
        (true, true) => {
            state.0 = ChordState::Flash {
                message: format!("both {kind:?} sides already spawned"),
                ttl_s: 2.0,
            };
        }
        (true, false) => spawn_side(kind, Side::Right, state, slots, teams),
        (false, true) => spawn_side(kind, Side::Left, state, slots, teams),
        (false, false) => state.0 = ChordState::AwaitingSpawnSide(kind),
    }
}

fn spawn_side(
    kind: RobotKind,
    side: Side,
    state: &mut ChordStateRes,
    slots: &mut SpawnSlots,
    teams: &TeamSides,
) {
    if slots.slots(kind)[side_idx(side)] {
        state.0 = ChordState::Flash {
            message: format!("{kind:?}/{} already spawned", side_name(side)),
            ttl_s: 2.0,
        };
        return;
    }
    launch_robot(kind, side, teams);
    slots.set(kind, side, true);
    state.0 = ChordState::Idle;
}

/// Spawn the manually-driven adversary. The sim creates the world
/// entity directly — no IPC, no child process. Subsequent ZQSD /
/// gamepad input drives its pose through `drive_adversary`.
pub fn spawn_adversary(
    state: &mut ChordStateRes,
    slots: &mut SpawnSlots,
    adversary: &mut AdversaryState,
    updates: &UpdatesTx,
    world: &SharedWorld,
    config: &crate::config::Config,
) {
    if adversary.id.is_some() || slots.adversary {
        state.0 = ChordState::Flash {
            message: "adversary already spawned".into(),
            ttl_s: 2.0,
        };
        return;
    }
    let id = "adversary".to_string();
    // Centre of the table in the asserv-aligned frame: X = x_max / 2,
    // Y = 0 (Y axis is centered around the Down-wall midpoint).
    let pose = Pose2D {
        x_mm: config.field.x_max_mm as f32 * 0.5,
        y_mm: 0.0,
        theta_rad: 0.0,
    };
    // Two-stage opponent silhouette: a wide base with a thin antenna on
    // top. Lidars below 350 mm hit the wide base; sensors between 350
    // and 430 mm only see the narrow antenna. `visible_segments`
    // filters per-primitive by `[z_base_mm, z_base_mm + height_mm]`.
    const BASE_RADIUS_MM: f32 = 225.0; // Ø450 mm
    const BASE_HEIGHT_MM: f32 = 350.0;
    const ANTENNA_RADIUS_MM: f32 = 35.0; // Ø70 mm
    const ANTENNA_HEIGHT_MM: f32 = 80.0;
    let total_height_mm = BASE_HEIGHT_MM + ANTENNA_HEIGHT_MM;
    let collision = Arc::new(vec![
        CollisionPrimitive::Cylinder {
            center_mm: [0.0, 0.0],
            radius_mm: BASE_RADIUS_MM,
            z_base_mm: 0.0,
            height_mm: BASE_HEIGHT_MM,
        },
        CollisionPrimitive::Cylinder {
            center_mm: [0.0, 0.0],
            radius_mm: ANTENNA_RADIUS_MM,
            z_base_mm: BASE_HEIGHT_MM,
            height_mm: ANTENNA_HEIGHT_MM,
        },
    ]);
    // bbox dimensions cover the worst-case footprint (the base) so any
    // consumer that only reads width/length still gets a conservative
    // outer envelope.
    let bbox_mm = BASE_RADIUS_MM * 2.0;
    let snap = EntitySnapshot {
        kind: EntityKind::Robot(RobotKind::Adversary),
        pose,
        width_mm: bbox_mm,
        length_mm: bbox_mm,
        height_above_table_mm: total_height_mm,
        collision: Some(collision),
    };
    world.0.spawn(id.clone(), snap);
    updates
        .0
        .send(WorldUpdate::Spawn {
            id: id.clone(),
            kind: EntityKind::Robot(RobotKind::Adversary),
            pose,
            width_mm: bbox_mm,
            length_mm: bbox_mm,
            body_height_mm: total_height_mm,
        })
        .ok();
    adversary.id = Some(id);
    adversary.pose = pose;
    slots.adversary = true;
    state.0 = ChordState::Idle;
    log::info!(
        "[adversary] spawned at ({:.0}, {:.0}) — drive with ZQSD or left stick",
        pose.x_mm, pose.y_mm
    );
}

fn despawn_adversary(adversary: &mut AdversaryState, updates: &UpdatesTx, world: &SharedWorld) {
    let Some(id) = adversary.id.take() else { return };
    world.0.remove(&id);
    updates.0.send(WorldUpdate::Despawn { id }).ok();
}

/// Startup-only: if main pre-seeded `--spawn adversary`, materialize the
/// in-process adversary now that the Bevy resources exist. Idempotent —
/// clears the request flag after the first run.
pub fn spawn_initial_adversary(
    mut state: ResMut<ChordStateRes>,
    mut slots: ResMut<SpawnSlots>,
    mut adversary: ResMut<AdversaryState>,
    updates: Res<UpdatesTx>,
    world: Res<SharedWorld>,
    config: Res<crate::app::SimConfigRes>,
) {
    if !slots.request_adversary_spawn {
        return;
    }
    slots.request_adversary_spawn = false;
    spawn_adversary(
        &mut state,
        &mut slots,
        &mut adversary,
        &updates,
        &world,
        &config.0,
    );
}

pub fn launch_robot(kind: RobotKind, side: Side, teams: &TeamSides) {
    // `s*` aliases target x86_64-unknown-linux-gnu (sim-native build);
    // the `r*` variants build for the ESP32 which makes no sense here.
    let alias = match kind {
        RobotKind::Galipeur => "sgalipeur",
        RobotKind::Pami => "spami",
        // Adversary is in-process, never launched as a child.
        RobotKind::Adversary => return,
    };
    let team = teams.team_of(side).unwrap_or("blue");
    let label = format!("{alias}/{}/{team}", side_name(side));
    // Side-suffixed id so the sim's `world` keeps the two robots
    // distinct (default is just the kind name, which collides when
    // both sides of the same kind spawn together).
    let robot_id = format!("{}-{}", match kind {
        RobotKind::Galipeur => "galipeur",
        RobotKind::Pami => "pami",
        RobotKind::Adversary => "adversary",
    }, side_name(side));
    match Command::new("cargo")
        .arg(alias)
        // Pass both: SIDE drives the sim Hello / colour pin; TEAM is
        // the current-year colour, useful for log labels and any strat
        // that wants to read it directly.
        .env("SIDE", side_name(side))
        .env("TEAM", team)
        .env("MECA_SIM_ROBOT_ID", &robot_id)
        .env("RUST_BACKTRACE", "1")
        .stdin(Stdio::null())
        // stdout/stderr inherit → child output lands in the sim console.
        .spawn()
    {
        Ok(child) => log::info!("spawned {label} (pid {}, id={robot_id})", child.id()),
        Err(e) => log::error!("failed to spawn {label}: {e}"),
    }
}

/// Pump ZQSD + gamepad input into the adversary's pose. Runs every
/// frame; no-ops while no adversary is spawned. The pose is forwarded
/// to both the world (so other robots' lidars see the new position)
/// and the bridge (so the renderer follows).
pub fn drive_adversary(
    keys: Option<Res<ButtonInput<Key>>>,
    keycodes: Option<Res<ButtonInput<KeyCode>>>,
    gamepads: Query<&Gamepad>,
    cameras: Query<&Transform, With<Camera3d>>,
    time: Res<Time>,
    mut adversary: ResMut<AdversaryState>,
    updates: Res<UpdatesTx>,
    world: Res<SharedWorld>,
    config: Res<crate::app::SimConfigRes>,
    chord: Res<ChordStateRes>,
) {
    let Some(_id) = adversary.id.clone() else { return };
    // While a chord menu is on the screen the operator is navigating
    // the menu, not driving — so an A press lands on "spawn
    // adversary" instead of "yaw left", and ZQSD letters that happen
    // to be valid menu choices don't get double-handled. Drive
    // resumes the moment the chord returns to Idle (Esc, completion,
    // or Flash expiry).
    if !matches!(chord.0, ChordState::Idle) {
        return;
    }

    // Linear / angular speeds tuned for a comfortable drive. The
    // operator can hold Shift via KeyCode (function key, not affected
    // by layout) for a slow-mo precision mode.
    const FAST_LIN_MM_S: f32 = 1500.0;
    const FAST_ANG_RAD_S: f32 = 3.0;
    const SLOW_LIN_MM_S: f32 = 400.0;
    const SLOW_ANG_RAD_S: f32 = 1.0;

    let slow = keycodes
        .as_ref()
        .map(|kc| kc.pressed(KeyCode::ShiftLeft) || kc.pressed(KeyCode::ShiftRight))
        .unwrap_or(false);
    let lin_speed = if slow { SLOW_LIN_MM_S } else { FAST_LIN_MM_S };
    let ang_speed = if slow { SLOW_ANG_RAD_S } else { FAST_ANG_RAD_S };

    // Logical input on a screen-relative 2D plane:
    //   forward = "into the scene" (away from the camera)
    //   right   = camera's right
    // These are unitless [-N, +N] amounts (sum of pressed keys / stick
    // values); the camera-relative basis is applied below.
    let mut forward = 0.0_f32;
    let mut right = 0.0_f32;
    let mut vt = 0.0_f32; // yaw, no camera mapping (axis-symmetric cylinder)

    if let Some(keys) = keys.as_ref() {
        if pressed_char(keys, "z") {
            forward += 1.0;
        }
        if pressed_char(keys, "s") {
            forward -= 1.0;
        }
        // ZQSD on AZERTY: Q is left of D in the logical layout.
        if pressed_char(keys, "q") {
            right -= 1.0;
        }
        if pressed_char(keys, "d") {
            right += 1.0;
        }
        // Optional yaw on A / E.
        if pressed_char(keys, "a") {
            vt += 1.0;
        }
        if pressed_char(keys, "e") {
            vt -= 1.0;
        }
    }

    // Gamepad overrides the keyboard contribution while the stick is
    // outside its dead zone, so the operator can mix smoothly. Stick
    // up = forward (camera-relative), stick right = right.
    if let Some(pad) = gamepads.iter().next() {
        let dz = |v: f32| if v.abs() < 0.12 { 0.0 } else { v };
        let lx = dz(pad.left_stick().x);
        let ly = dz(pad.left_stick().y);
        let rx = dz(pad.right_stick().x);
        if lx != 0.0 || ly != 0.0 {
            forward = ly;
            right = lx;
        }
        if rx != 0.0 {
            vt = -rx;
        }
    }

    if forward == 0.0 && right == 0.0 && vt == 0.0 {
        return;
    }

    // Camera-relative basis on the table plane (Bevy XZ, Y is up).
    // `forward_xz` is the camera-look projected onto the floor;
    // `right_xz` is rotated −90° around +Y from it. When the camera
    // is straight down (gimbal-locked, very steep pitch) `forward_xz`
    // would be ~0 — fall back to world −Z so input still produces a
    // sensible direction.
    let cam_t = cameras.iter().next();
    let (forward_xz, right_xz) = if let Some(t) = cam_t {
        let f = t.forward();
        let mut fx = Vec3::new(f.x, 0.0, f.z);
        if fx.length_squared() < 1e-4 {
            // Pitched straight down (or up) — pick a stable fallback.
            // `Transform::up` projected on XZ gives the ceiling-side
            // direction, which for an orbit camera looking down from
            // above maps to the screen-up direction.
            let up = t.up();
            fx = Vec3::new(up.x, 0.0, up.z);
            if fx.length_squared() < 1e-4 {
                fx = Vec3::new(0.0, 0.0, -1.0);
            }
        }
        let fx = fx.normalize();
        // right = rotate `forward` by −π/2 around +Y. Using axis-Y
        // rotation: (x, 0, z) → (z, 0, −x). Sanity: with f = (0,0,−1)
        // (camera looking down −Z), right = (−1, 0, 0)? That's wrong;
        // right should be +X. The correct mapping for a right-handed
        // Y-up frame is `right = (−forward.z, 0, forward.x)`:
        // with f = (0,0,−1), right = (1, 0, 0) ✓.
        let rx = Vec3::new(-fx.z, 0.0, fx.x);
        (fx, rx)
    } else {
        (Vec3::new(0.0, 0.0, -1.0), Vec3::new(1.0, 0.0, 0.0))
    };

    // Compose the camera-relative move in Bevy world XZ, then map to
    // sim world XY. After the asserv-aligned refactor the sim ↔ bevy
    // mapping is a proper rotation: sim.x = bevy.x, sim.y = −bevy.z
    // (matches `body_pos_to_bevy`). Normalising first keeps "Z + D"
    // diagonals at the same speed as a single axis press.
    let bevy_dir = (forward_xz * forward + right_xz * right).normalize_or_zero();
    let dt = time.delta_secs();
    let theta = adversary.pose.theta_rad;
    let dx = bevy_dir.x * lin_speed * dt;
    let dy = -bevy_dir.z * lin_speed * dt;
    let dt_theta = vt * ang_speed * dt;

    let x_max = config.0.field.x_max_mm as f32;
    let y_half = config.0.field.y_half_mm as f32;
    let new_x = (adversary.pose.x_mm + dx).clamp(0.0, x_max);
    let new_y = (adversary.pose.y_mm + dy).clamp(-y_half, y_half);
    let new_theta = wrap_pi(theta + dt_theta);

    adversary.pose.x_mm = new_x;
    adversary.pose.y_mm = new_y;
    adversary.pose.theta_rad = new_theta;

    if let Some(id) = adversary.id.as_ref() {
        world.0.update_pose(id, adversary.pose);
        updates
            .0
            .send(WorldUpdate::UpdatePose {
                id: id.clone(),
                pose: adversary.pose,
            })
            .ok();
    }
}

fn wrap_pi(t: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    let mut t = t % TAU;
    if t > PI {
        t -= TAU;
    } else if t < -PI {
        t += TAU;
    }
    t
}

/// Rebuild the hint overlay whenever the chord state changes. When
/// `Idle` the overlay is despawned. Uses HUD tokens + widgets so it
/// matches the rest of the look (mono font, panel chrome, kbd badges).
pub fn render_chord_overlay(
    mut commands: Commands,
    state: Res<ChordStateRes>,
    fonts: Option<Res<crate::hud::tokens::HudFonts>>,
    existing: Query<Entity, With<ChordOverlay>>,
) {
    if !state.is_changed() {
        return;
    }
    for e in &existing {
        commands.entity(e).despawn();
    }
    if matches!(state.0, ChordState::Idle) {
        return;
    }
    let Some(fonts) = fonts else { return };

    let backdrop = commands
        .spawn((
            ChordOverlay,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(0.0),
                left: Val::Px(0.0),
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
            ZIndex(60),
        ))
        .id();

    commands.entity(backdrop).with_children(|p| match &state.0 {
        ChordState::Idle => {}
        ChordState::AwaitingSpawnKind => {
            spawn_chord_card(p, &fonts, "spawn", &[
                ("G", "galipeur"),
                ("P", "pami"),
                ("A", "adversary (manual)"),
            ]);
        }
        ChordState::AwaitingSpawnSide(RobotKind::Galipeur) => {
            spawn_chord_card(p, &fonts, "spawn galipeur", &[
                ("L", "left"),
                ("R", "right"),
            ]);
        }
        ChordState::AwaitingSpawnSide(RobotKind::Pami) => {
            spawn_chord_card(p, &fonts, "spawn pami", &[
                ("L", "left"),
                ("R", "right"),
            ]);
        }
        ChordState::AwaitingSpawnSide(RobotKind::Adversary) => {
            spawn_chord_card(p, &fonts, "spawn adversary", &[(
                "A",
                "(no side — press A again)",
            )]);
        }
        ChordState::Flash { message, .. } => {
            spawn_flash_card(p, &fonts, message);
        }
    });
}

/// Modal card used by the chord prompt screens (AwaitingSpawnKind /
/// AwaitingSpawnSide). Title + N keyboard hint rows + a final
/// `[Esc] cancel` row.
fn spawn_chord_card(
    parent: &mut ChildSpawnerCommands,
    fonts: &crate::hud::tokens::HudFonts,
    title: &str,
    items: &[(&str, &str)],
) {
    use crate::hud::tokens;
    use crate::hud::widgets::{card, kbd};
    let (bg, border) = card::chrome();
    parent
        .spawn((
            Node {
                border: UiRect::all(Val::Px(1.0)),
                border_radius: card::radius_panel(),
                padding: UiRect::axes(Val::Px(20.0), Val::Px(16.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(10.0),
                min_width: Val::Px(280.0),
                ..default()
            },
            bg,
            border,
        ))
        .with_children(|card| {
            // Section title with leading accent dot.
            card.spawn(Node {
                column_gap: Val::Px(6.0),
                align_items: AlignItems::Center,
                ..default()
            })
            .with_children(|t| {
                t.spawn((
                    Node {
                        width: Val::Px(6.0),
                        height: Val::Px(6.0),
                        border_radius: BorderRadius::all(Val::Px(50.0)),
                        ..default()
                    },
                    BackgroundColor(tokens::ACCENT),
                ));
                t.spawn((
                    Text::new(title),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 11.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
            });
            // Items.
            for (k, d) in items {
                card.spawn(Node {
                    column_gap: Val::Px(8.0),
                    align_items: AlignItems::Center,
                    padding: UiRect::axes(Val::Px(0.0), Val::Px(2.0)),
                    ..default()
                })
                .with_children(|row| {
                    row.spawn(Node {
                        min_width: Val::Px(28.0),
                        ..default()
                    })
                    .with_children(|wrap| {
                        kbd::spawn(wrap, fonts, k, false);
                    });
                    row.spawn((
                        Text::new(*d),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 12.0,
                            ..default()
                        },
                        TextColor(tokens::TEXT),
                    ));
                });
            }
            // Cancel hint with a divider above.
            card.spawn((
                Node {
                    margin: UiRect::top(Val::Px(4.0)),
                    padding: UiRect::top(Val::Px(8.0)),
                    border: UiRect {
                        top: Val::Px(1.0),
                        ..default()
                    },
                    column_gap: Val::Px(8.0),
                    align_items: AlignItems::Center,
                    ..default()
                },
                BorderColor::all(tokens::BORDER),
            ))
            .with_children(|row| {
                kbd::spawn(row, fonts, "Esc", true);
                row.spawn((
                    Text::new("cancel"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 11.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
            });
        });
}

/// One-line flash card with a coloured accent strip on the left to
/// signal severity (refusal/error vs info).
fn spawn_flash_card(
    parent: &mut ChildSpawnerCommands,
    fonts: &crate::hud::tokens::HudFonts,
    message: &str,
) {
    use crate::hud::tokens;
    use crate::hud::widgets::card;
    let lower = message.to_ascii_lowercase();
    let (accent, glyph) = if lower.contains("fail") || lower.contains("error") {
        (tokens::RED, "\u{2715}") // ✕
    } else if lower.contains("refus")
        || lower.contains("already")
        || lower.contains("disabled")
    {
        (tokens::YELLOW, "!")
    } else {
        (tokens::ACCENT_HOT, "i")
    };
    let (bg, _) = card::chrome();
    parent
        .spawn((
            Node {
                border: UiRect::left(Val::Px(3.0)),
                border_radius: card::radius_panel(),
                padding: UiRect::axes(Val::Px(18.0), Val::Px(14.0)),
                column_gap: Val::Px(10.0),
                align_items: AlignItems::Center,
                max_width: Val::Px(560.0),
                ..default()
            },
            bg,
            BorderColor::all(accent),
        ))
        .with_children(|card| {
            card.spawn((
                Text::new(glyph),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 14.0,
                    weight: bevy::text::FontWeight::BOLD,
                    ..default()
                },
                TextColor(accent),
            ));
            card.spawn((
                Text::new(message),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 13.0,
                    ..default()
                },
                TextColor(tokens::TEXT),
            ));
        });
}
