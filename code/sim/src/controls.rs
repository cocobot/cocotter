//! Keyboard controls: chord-based shortcuts with an on-screen hint
//! overlay driven by a small state machine.
//!
//! - `K`                kill-all: shutdown every active IPC connection.
//! - `S` `G`/`P` `B`/`Y` spawn a robot (`cargo rgalipeur` / `cargo rpami`)
//!                       and inherit its stdout/stderr into the sim console.
//! - `Esc`              cancel an in-progress chord.
//!
//! The active chord state drives a centred hint overlay so the user sees
//! which keys are available at each step.

use std::net::Shutdown;
use std::os::unix::net::UnixStream;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use sim_protocol::{send_msg, RobotKind, SimMsgS2C};

use crate::config::{Side, TeamSides};

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
}

#[derive(Default, Clone, Debug, PartialEq)]
pub enum ChordState {
    #[default]
    Idle,
    AwaitingSpawnKind,
    AwaitingSpawnTeam(RobotKind),
    /// Transient message shown on the chord overlay, reverts to `Idle`
    /// after `ttl_s` seconds. Used to explain refusals like "both teams
    /// already spawned" without freezing the input.
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

/// Which `(kind, team)` combinations the sim has explicitly launched.
/// `kill_all` clears every flag; there is no tracking of externally-
/// spawned robots (those connecting via `cargo run` from a terminal),
/// which is acceptable for the chord UX — the user is already in charge
/// of what they started themselves.
#[derive(Resource, Default, Clone)]
pub struct SpawnSlots {
    pub galipeur: [bool; 2],
    pub pami: [bool; 2],
}

impl SpawnSlots {
    fn slots(&self, kind: RobotKind) -> [bool; 2] {
        match kind {
            RobotKind::Galipeur => self.galipeur,
            RobotKind::Pami => self.pami,
        }
    }

    fn set(&mut self, kind: RobotKind, team_idx: usize, val: bool) {
        let arr = match kind {
            RobotKind::Galipeur => &mut self.galipeur,
            RobotKind::Pami => &mut self.pami,
        };
        arr[team_idx] = val;
    }

    pub fn clear_all(&mut self) {
        self.galipeur = [false; 2];
        self.pami = [false; 2];
    }
}

fn team_idx(team: &str) -> usize {
    if team == "yellow" { 1 } else { 0 }
}

#[derive(Component)]
pub struct ChordOverlay;

pub fn chord_input(
    keys: Option<Res<ButtonInput<KeyCode>>>,
    time: Res<Time>,
    mut state: ResMut<ChordStateRes>,
    mut slots: ResMut<SpawnSlots>,
    conns: Res<ConnRegistry>,
    teams: Res<TeamSidesRes>,
) {
    // Countdown on any active Flash state — reverts to Idle when expired.
    if let ChordState::Flash { ttl_s, .. } = &mut state.0 {
        *ttl_s -= time.delta_secs();
        if *ttl_s <= 0.0 {
            state.0 = ChordState::Idle;
        }
    }

    let Some(keys) = keys else { return };

    if keys.just_pressed(KeyCode::Escape) {
        if state.0 != ChordState::Idle {
            state.0 = ChordState::Idle;
        }
        return;
    }

    match state.0.clone() {
        ChordState::Idle => {
            if keys.just_pressed(KeyCode::KeyK) {
                let n = conns.kill_all();
                slots.clear_all();
                log::info!("kill-all: disconnected {n} robot(s)");
            } else if keys.just_pressed(KeyCode::KeyQ) {
                // Also kill the robots on the way out so they don't
                // spam errors on the now-closed socket.
                let _ = conns.kill_all();
                log::info!("quit requested (Q)");
                std::process::exit(0);
            } else if keys.just_pressed(KeyCode::KeyS) {
                state.0 = ChordState::AwaitingSpawnKind;
            }
        }
        ChordState::AwaitingSpawnKind => {
            if keys.just_pressed(KeyCode::KeyG) {
                pick_kind(RobotKind::Galipeur, &mut state, &mut slots, &teams.0);
            } else if keys.just_pressed(KeyCode::KeyP) {
                pick_kind(RobotKind::Pami, &mut state, &mut slots, &teams.0);
            }
        }
        ChordState::AwaitingSpawnTeam(kind) => {
            if keys.just_pressed(KeyCode::KeyB) {
                spawn_team(kind, "blue", &mut state, &mut slots, &teams.0);
            } else if keys.just_pressed(KeyCode::KeyY) {
                spawn_team(kind, "yellow", &mut state, &mut slots, &teams.0);
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

fn pick_kind(
    kind: RobotKind,
    state: &mut ChordStateRes,
    slots: &mut SpawnSlots,
    teams: &TeamSides,
) {
    let [blue, yellow] = slots.slots(kind);
    match (blue, yellow) {
        (true, true) => {
            state.0 = ChordState::Flash {
                message: format!("both {kind:?} teams already spawned"),
                ttl_s: 2.0,
            };
        }
        (true, false) => spawn_team(kind, "yellow", state, slots, teams),
        (false, true) => spawn_team(kind, "blue", state, slots, teams),
        (false, false) => state.0 = ChordState::AwaitingSpawnTeam(kind),
    }
}

fn spawn_team(
    kind: RobotKind,
    team: &str,
    state: &mut ChordStateRes,
    slots: &mut SpawnSlots,
    teams: &TeamSides,
) {
    let idx = team_idx(team);
    if slots.slots(kind)[idx] {
        state.0 = ChordState::Flash {
            message: format!("{kind:?}/{team} already spawned"),
            ttl_s: 2.0,
        };
        return;
    }
    launch_robot(kind, team, teams);
    slots.set(kind, idx, true);
    state.0 = ChordState::Idle;
}

fn launch_robot(kind: RobotKind, team: &str, teams: &TeamSides) {
    // `s*` aliases target x86_64-unknown-linux-gnu (sim-native build);
    // the `r*` variants build for the ESP32 which makes no sense here.
    let alias = match kind {
        RobotKind::Galipeur => "sgalipeur",
        RobotKind::Pami => "spami",
    };
    let side = teams.side_of(team).unwrap_or(Side::Left);
    let side_name = match side {
        Side::Left => "left",
        Side::Right => "right",
    };
    let label = format!("{alias}/{team}/{side_name}");
    match Command::new("cargo")
        .arg(alias)
        // Robot binaries may read either TEAM (colour) or SIDE (which
        // side of the table). Both are passed; pick whichever fits the
        // current strat.
        .env("TEAM", team)
        .env("SIDE", side_name)
        .stdin(Stdio::null())
        // stdout/stderr inherit → child output lands in the sim console.
        .spawn()
    {
        Ok(child) => log::info!("spawned {label} (pid {})", child.id()),
        Err(e) => log::error!("failed to spawn {label}: {e}"),
    }
}

/// Rebuild the hint overlay whenever the chord state changes. When `Idle`
/// the overlay is despawned so nothing obstructs the scene.
pub fn render_chord_overlay(
    mut commands: Commands,
    state: Res<ChordStateRes>,
    existing: Query<Entity, With<ChordOverlay>>,
) {
    if !state.is_changed() {
        return;
    }
    for e in &existing {
        commands.entity(e).despawn();
    }
    let body: String = match &state.0 {
        ChordState::Idle => return,
        ChordState::AwaitingSpawnKind => {
            "Spawn:\n  G  galipeur\n  P  pami\n\n  Esc  cancel".into()
        }
        ChordState::AwaitingSpawnTeam(RobotKind::Galipeur) => {
            "Spawn galipeur:\n  B  blue\n  Y  yellow\n\n  Esc  cancel".into()
        }
        ChordState::AwaitingSpawnTeam(RobotKind::Pami) => {
            "Spawn pami:\n  B  blue\n  Y  yellow\n\n  Esc  cancel".into()
        }
        ChordState::Flash { message, .. } => message.clone(),
    };
    // Full-screen flex container so the hint stays centred regardless of
    // window size. The container itself has no background and no pointer
    // interactions, only the inner card is visible.
    commands
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
        ))
        .with_children(|root| {
            root.spawn((
                Node {
                    padding: UiRect::all(Val::Px(24.0)),
                    flex_direction: FlexDirection::Column,
                    ..default()
                },
                BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.8)),
            ))
            .with_children(|card| {
                card.spawn((
                    Text::new(body),
                    TextFont { font_size: 28.0, ..default() },
                    TextColor(Color::WHITE),
                ));
            });
        });
}
