//! New HUD overlay — pixel-perfect port of the React mockup
//! (`HUD live standalone.html`, variant "C · Hybrid"). See
//! `~/.claude/plans/on-va-rework-entirement-elegant-lollipop.md` for the
//! phase-by-phase rollout.

use bevy::prelude::*;

pub mod cheatsheet;
pub mod dock;
pub mod footer;
pub mod header;
pub mod log_capture;
pub mod log_tail;
pub mod logs_panel;
pub mod robots_panel;
pub mod style;
pub mod tokens;
pub mod watch_panel;
pub mod widgets;

use crate::app::Headless;
use tokens::HudFonts;

// ─── Match-state mock (HUD-side; the real `controls::MatchState` is
//     bridged in Phase 10) ─────────────────────────────────────────────

#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum HudMatchState {
    #[default]
    Pregame,
    /// Starter (`tirette`) inserted — waiting for the second T press
    /// to remove it and start the match. Spawning new robots is
    /// disabled by the sim in this state.
    Starter,
    Running,
    Ended,
}

impl HudMatchState {
    pub fn next(self) -> Self {
        match self {
            Self::Pregame => Self::Starter,
            Self::Starter => Self::Running,
            Self::Running => Self::Ended,
            Self::Ended => Self::Pregame,
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    #[default]
    Left,
    Right,
}

impl Side {
    pub fn flip(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
        }
    }
}

/// Mock data driving every HUD widget. Replaced in Phase 10 by a
/// per-source aggregator wired to the actual sim events; for now we
/// just hand-tweak with F1 / F2 / F3.
#[derive(Resource, Clone, Debug)]
pub struct HudMockData {
    pub match_state: HudMatchState,
    pub elapsed_secs: f32,
    pub match_secs: f32,
    pub score_left: u32,
    pub score_right: u32,
    pub our_side: Side,
    pub ipc_lat_ms: f32,
    pub ipc_pkts_per_s: u32,
    pub ipc_drops: u32,
    pub robots_online: u32,
    pub robots_total: u32,
    pub frame_fps: u32,
    pub asserv_ms: f32,
}

// ─── Log model (used by Phase 5 LogsPanel) ─────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    pub fn idx(self) -> usize {
        match self {
            LogLevel::Debug => 0,
            LogLevel::Info => 1,
            LogLevel::Warn => 2,
            LogLevel::Error => 3,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            LogLevel::Debug => "debug",
            LogLevel::Info => "info",
            LogLevel::Warn => "warn",
            LogLevel::Error => "error",
        }
    }
    pub fn glyph(self) -> &'static str {
        match self {
            LogLevel::Debug => "·",
            LogLevel::Info => "i",
            LogLevel::Warn => "!",
            LogLevel::Error => "\u{2715}", // ✕
        }
    }
    pub fn color(self) -> bevy::prelude::Color {
        match self {
            LogLevel::Debug => bevy::prelude::Color::srgb(0.490, 0.541, 0.541), // #7d8a8a
            LogLevel::Info => bevy::prelude::Color::srgb(0.608, 0.823, 0.651), // #9bd2a6
            LogLevel::Warn => tokens::YELLOW,
            LogLevel::Error => tokens::RED,
        }
    }
}

pub const LOG_LEVELS: [LogLevel; 4] = [
    LogLevel::Debug,
    LogLevel::Info,
    LogLevel::Warn,
    LogLevel::Error,
];

/// Channel names kept in `LogEntry.channel` for display, but the
/// LogsPanel no longer filters by channel — only by source. Phase
/// 10d-rework.
pub const LOG_CHANNELS: [&str; 7] = [
    "core",
    "ipc",
    "boot",
    "strat",
    "asserv",
    "lidar",
    "actuators",
];

/// Three sources surfaced by the HUD filter UI. Anything from a
/// non-matching module is mapped to `sim`.
pub const LOG_SOURCES: [&str; 3] = ["sim", "galipeur", "pami"];

#[derive(Clone, Debug)]
pub struct LogEntry {
    pub t: f32,
    pub src: String,
    pub channel: String,
    pub level: LogLevel,
    pub msg: String,
}

/// Filter state + log buffer. Matches the LogsPanel internal state in
/// `hud_02.jsx:229-244`. Kept as a separate resource (not folded into
/// `HudMockData`) so live-log producers in Phase 10 can push without
/// touching mock-only fields.
#[derive(Resource, Clone)]
pub struct HudLogState {
    pub entries: Vec<LogEntry>,
    pub paused: bool,
    pub levels: [bool; 4],
    /// One bool per source in `LOG_SOURCES` order
    /// (`sim` / `galipeur` / `pami`).
    pub sources: [bool; 3],
    /// Free-text filter applied to `msg` + `src`. Empty = no filter.
    pub filter_query: String,
}

impl Default for HudLogState {
    fn default() -> Self {
        Self {
            entries: seed_logs(),
            paused: false,
            levels: [true; 4],
            sources: [true; 3],
            filter_query: String::new(),
        }
    }
}

/// Tracks whether the LogsPanel filter input has keyboard focus.
/// Clicking the input acquires focus, clicking elsewhere releases it.
#[derive(Resource, Default)]
pub struct LogsFilterFocus(pub bool);

/// Visibility flag for the cheatsheet modal (Phase 9).
#[derive(Resource, Default)]
pub struct CheatsheetVisible(pub bool);

/// Single keyboard shortcut shown in the cheatsheet. Mirrors
/// `SHORTCUTS` in `hud_01.jsx:50-68`.
#[derive(Clone, Copy)]
pub struct Shortcut {
    pub key: &'static str,
    pub desc: &'static str,
    pub scope: ShortcutScope,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ShortcutScope {
    Hud,
    Match,
    View,
}

/// Cheatsheet contents — every entry must match a real binding in
/// `sim/src/`. Audited against:
///   - controls.rs::chord_input          (T / R / K / N / X + chord
///                                        sub-states G/P/A then L/R,
///                                        Esc cancels)
///   - dock.rs::dock_keyboard_input      (1 / 2 / 3, Esc closes)
///   - cheatsheet.rs::handle_keyboard    (?)
///   - app.rs (overlay toggles)          (H / C / V / L)
///   - controls.rs::drive_adversary      (ZQSD / gamepad)
///   - PanOrbitCamera plugin             (LMB / RMB / wheel)
pub const SHORTCUTS: &[Shortcut] = &[
    // HUD / dock — what the user toggles to inspect things.
    Shortcut { key: "?",     desc: "cheatsheet",       scope: ShortcutScope::Hud },
    Shortcut { key: "1",     desc: "logs panel",       scope: ShortcutScope::Hud },
    Shortcut { key: "2",     desc: "watch panel",      scope: ShortcutScope::Hud },
    Shortcut { key: "3",     desc: "robots panel",     scope: ShortcutScope::Hud },
    Shortcut { key: "Esc",   desc: "close dock / modal / chord", scope: ShortcutScope::Hud },
    // Match / spawn.
    Shortcut { key: "T",     desc: "insert / remove starter",     scope: ShortcutScope::Match },
    Shortcut { key: "N",     desc: "spawn → G/P/A · L/R",         scope: ShortcutScope::Match },
    Shortcut { key: "K",     desc: "kill all robots",             scope: ShortcutScope::Match },
    Shortcut { key: "R",     desc: "reset world",                 scope: ShortcutScope::Match },
    Shortcut { key: "X",     desc: "quit sim",                    scope: ShortcutScope::Match },
    // 3D view + overlays.
    Shortcut { key: "LMB",    desc: "orbit camera",        scope: ShortcutScope::View },
    Shortcut { key: "RMB",    desc: "pan camera",          scope: ShortcutScope::View },
    Shortcut { key: "Wheel",  desc: "zoom",                scope: ShortcutScope::View },
    Shortcut { key: "H",      desc: "collision overlay",   scope: ShortcutScope::View },
    Shortcut { key: "C",      desc: "toggle humans",       scope: ShortcutScope::View },
    Shortcut { key: "V",      desc: "debug volumes",       scope: ShortcutScope::View },
    Shortcut { key: "L",      desc: "lidar beam viz",      scope: ShortcutScope::View },
    Shortcut { key: "ZQSD",   desc: "drive adversary",     scope: ShortcutScope::View },
];

// ─── Watch panel KV model (Phase 6) ────────────────────────────────

/// One key/value pair surfaced by a robot to the HUD. Mirrors
/// `FAKE_KV` from `hud_01.jsx:30-40`. Phase 10 will replace the seed
/// with a live `SimMsgC2S::HudKv` channel.
#[derive(Clone, Debug)]
pub struct HudKv {
    pub key: &'static str,
    pub value: &'static str,
    pub src: &'static str,
}

#[derive(Resource, Clone, Default)]
pub struct HudKvState {
    pub entries: Vec<HudKv>,
}

pub fn seed_kv() -> Vec<HudKv> {
    vec![
        HudKv { key: "current_target",  value: "jardiniere_W",     src: "galipeur" },
        HudKv { key: "strat_node",      value: "GoTo[8/14]",       src: "galipeur" },
        HudKv { key: "battery",         value: "12.4 V",           src: "galipeur" },
        HudKv { key: "asserv_err",      value: "4 mm · 0.02 rad",  src: "galipeur" },
        HudKv { key: "lidar_pts",       value: "3892 /s",          src: "galipeur" },
        HudKv { key: "claw_state",      value: "open · idle",      src: "picotter" },
        HudKv { key: "ipc_latency",     value: "1.2 ms",           src: "sim"      },
        HudKv { key: "ipc_pkts",        value: "412 /s · 0 drop",  src: "sim"      },
    ]
}

// ─── Robots panel model (Phase 7) ──────────────────────────────────

#[derive(Clone, Debug)]
pub struct HudRobot {
    pub id: String,
    pub kind: &'static str,
    pub side: Side,
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub online: bool,
    /// `true` for the local-only adversary driven via ZQSD/gamepad
    /// (mockup `hud_01.jsx:47`).
    pub fake: bool,
    /// Driver hint shown next to the FAKE chip — "ZQSD", "GAMEPAD", etc.
    pub driver: Option<&'static str>,
}

#[derive(Resource, Clone, Default)]
pub struct HudRobotsState {
    pub robots: Vec<HudRobot>,
}

pub fn seed_robots() -> Vec<HudRobot> {
    // Empty by default — Phase 10c populates this from the live World
    // snapshot. The seed is kept as `vec![]` so headless / startup
    // before any robot connects shows an empty robots panel rather
    // than fake mock entries.
    vec![]
}

/// Empty by default — Phase 10d's logger captures live records into
/// the buffer at runtime.
fn seed_logs() -> Vec<LogEntry> {
    Vec::new()
}

impl Default for HudMockData {
    fn default() -> Self {
        Self {
            match_state: HudMatchState::Pregame,
            elapsed_secs: 0.0,
            match_secs: 100.0,
            score_left: 12,
            score_right: 7,
            our_side: Side::Left,
            ipc_lat_ms: 1.2,
            ipc_pkts_per_s: 412,
            ipc_drops: 0,
            robots_online: 4,
            robots_total: 5,
            frame_fps: 144,
            asserv_ms: 1.2,
        }
    }
}

#[derive(Component)]
pub struct HudRoot;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(HudMockData::default())
            .insert_resource(HudLogState::default())
            .insert_resource(HudKvState { entries: seed_kv() })
            .insert_resource(HudRobotsState { robots: seed_robots() })
            .insert_resource(LogsFilterFocus::default())
            .insert_resource(CheatsheetVisible::default())
            .insert_resource(logs_panel::LogsStickyFrames::default())
            .add_systems(Startup, setup_hud)
            .add_systems(
                Startup,
                (
                    header::setup_header.after(setup_hud),
                    footer::setup_footer.after(setup_hud),
                    dock::setup_dock.after(setup_hud),
                    log_tail::setup_log_tail.after(setup_hud),
                    cheatsheet::setup_cheatsheet.after(setup_hud),
                ),
            )
            .add_systems(
                Update,
                (
                    advance_mock_clock,
                    dev_keys_cycle_mock,
                    bind_match_state,
                    robots_panel::bind_robots,
                    robots_panel::refresh_robots_ui,
                    header::update_header_text,
                    header::update_header_nodes,
                    footer::update_footer_text,
                ),
            )
            // The logs pipeline runs in strict order, with command
            // flushes between, so each stage sees the latest state:
            //   1. drain channel → state.entries
            //   2. respawn rows from state (also auto-scrolls on grow)
            //   3. apply level/source/query filter to the freshly
            //      spawned rows
            // Without `.chain()` the three could interleave and the
            // first frame after boot would either skip the auto-scroll
            // or apply the filter to about-to-be-despawned rows.
            .add_systems(
                Update,
                (
                    log_capture::drain_into_log_state,
                    logs_panel::refresh_logs_ui,
                    logs_panel::refresh_filter_visuals,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (
                    dock::dock_button_input,
                    dock::dock_keyboard_input,
                    dock::update_dock_visuals,
                    dock::camera_mute_when_over_dock,
                    dock::update_cursor_icon,
                    logs_panel::handle_filter_clicks,
                    logs_panel::handle_filter_keyboard,
                    logs_panel::handle_scroll_wheel,
                    logs_panel::auto_scroll_on_dock_open,
                    logs_panel::clamp_logs_scroll_position,
                    watch_panel::handle_scroll_wheel,
                    robots_panel::handle_scroll_wheel,
                    log_tail::update_log_tail,
                ),
            )
            .add_systems(
                Update,
                (
                    cheatsheet::handle_keyboard,
                    cheatsheet::handle_clicks,
                    cheatsheet::refresh_visibility,
                    footer::handle_cheatsheet_btn,
                ),
            );
    }
}

fn setup_hud(mut commands: Commands, asset_server: Res<AssetServer>, headless: Res<Headless>) {
    if headless.0 {
        return;
    }

    commands.insert_resource(HudFonts::load(&asset_server));

    commands.spawn((
        HudRoot,
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(0.0),
            left: Val::Px(0.0),
            right: Val::Px(0.0),
            bottom: Val::Px(0.0),
            ..default()
        },
    ));
}

/// In RUNNING state, advance the elapsed mock clock. Stops at `match_secs`
/// then auto-flips to ENDED to demo all 3 visual states without manual
/// keyboard fiddling.
fn advance_mock_clock(time: Res<Time>, mut data: ResMut<HudMockData>) {
    if data.match_state == HudMatchState::Running {
        data.elapsed_secs += time.delta_secs();
        if data.elapsed_secs >= data.match_secs {
            data.elapsed_secs = data.match_secs;
            data.match_state = HudMatchState::Ended;
        }
    }
}

/// Dev-only keyboard helpers for *mock-only* fields. F1 (match state)
/// is now driven by `controls::MatchStateLock` — see `bind_match_state`.
///   F2 → flip our_side
///   F3 → +1 left score · F4 → +1 right score
fn dev_keys_cycle_mock(
    keys: Res<ButtonInput<KeyCode>>,
    focus: Res<LogsFilterFocus>,
    mut data: ResMut<HudMockData>,
) {
    if focus.0 {
        return;
    }
    if keys.just_pressed(KeyCode::F2) {
        data.our_side = data.our_side.flip();
    }
    if keys.just_pressed(KeyCode::F3) {
        data.score_left = data.score_left.saturating_add(1);
    }
    if keys.just_pressed(KeyCode::F4) {
        data.score_right = data.score_right.saturating_add(1);
    }
}

/// Phase 10a — bind HUD match state to `controls::MatchStateLock`.
/// Maps the sim's 3 states (Pregame / StarterInserted / Running) plus
/// the HUD-internal `Ended` (timer expired):
///
///   sim Pregame       → HUD Pregame  (reset elapsed at edge)
///   sim StarterInsert → HUD Starter
///   sim Running       → HUD Running  (reset elapsed at edge)
///   …                 → HUD Ended    (kept until sim goes Pregame)
fn bind_match_state(
    match_lock: Res<crate::controls::MatchStateLock>,
    mut data: ResMut<HudMockData>,
) {
    use crate::controls::MatchState as Sim;
    let target = match match_lock.get() {
        Sim::Pregame => HudMatchState::Pregame,
        Sim::StarterInserted => HudMatchState::Starter,
        Sim::Running => HudMatchState::Running,
    };

    // HUD `Ended` sticks until sim returns to Pregame — once the timer
    // expires we don't want a sim that's still `Running` to flip the
    // HUD back.
    if data.match_state == HudMatchState::Ended && target != HudMatchState::Pregame {
        return;
    }

    if target != data.match_state {
        // Reset the elapsed timer at every state edge except going
        // Starter → Running (we want elapsed to start counting at the
        // match start, which is when we ENTER Running).
        let entered_running = target == HudMatchState::Running;
        if entered_running || target == HudMatchState::Pregame {
            data.elapsed_secs = 0.0;
        }
        data.match_state = target;
    }
}
