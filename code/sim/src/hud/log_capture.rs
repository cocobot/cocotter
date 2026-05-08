//! Phase 10d — capture every `log::Record` into the HUD's
//! `HudLogState` ringbuffer while still mirroring to stderr (so
//! existing console workflows keep working).
//!
//! Design:
//!  - Replace the `env_logger::init` call with a custom wrapper that
//!    delegates printing to env_logger but ALSO sends a `LogEntry`
//!    over a `flume` channel.
//!  - The Bevy side keeps a `Receiver` resource and drains it each
//!    frame into `HudLogState.entries`, capping at 200 entries (same
//!    cap as the mockup's `LiveApp` in `hud_04.jsx:131`).
//!
//! Module path → channel/source mapping:
//!  - `<crate>::strat`          → channel "strat"
//!  - `<crate>::asserv` etc.    → matching channel name
//!  - `sim::server` / `bridge`  → channel "ipc"
//!  - first segment of target   → source ("sim", "galipeur",
//!                                "picotter", default "core")

use std::time::Instant;

use bevy::prelude::*;

use super::{HudLogState, LogEntry, LogLevel, LOG_CHANNELS};

// (LOG_CHANNELS still used to map module path → channel for display
// purposes; the LogsPanel filter UI no longer surfaces channels.)

/// One captured log line in transit between the global logger and the
/// Bevy frame system.
#[derive(Clone, Debug)]
pub struct CapturedLog {
    pub t: f32,
    pub src: String,
    pub channel: String,
    pub level: LogLevel,
    pub msg: String,
}

#[derive(Resource)]
pub struct LogCaptureRx(pub flume::Receiver<CapturedLog>);

/// Install the global `log` logger. Mirrors env_logger's stderr output
/// AND sends every record through the returned channel sender. Must be
/// called *instead* of `env_logger::init`, before any sim code logs.
pub fn install(default_filter: &str) -> flume::Receiver<CapturedLog> {
    let (tx, rx) = flume::unbounded::<CapturedLog>();
    let env_logger = env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or(default_filter),
    )
    .build();
    let max_level = env_logger.filter();
    let combo = ComboLogger {
        inner: env_logger,
        tx,
        start: Instant::now(),
    };
    log::set_boxed_logger(Box::new(combo)).expect("logger already set");
    log::set_max_level(max_level);
    rx
}

struct ComboLogger {
    inner: env_logger::Logger,
    tx: flume::Sender<CapturedLog>,
    start: Instant,
}

impl log::Log for ComboLogger {
    fn enabled(&self, m: &log::Metadata) -> bool {
        self.inner.enabled(m)
    }
    fn log(&self, record: &log::Record) {
        // Stderr-side mirror first.
        self.inner.log(record);
        if !self.inner.enabled(record.metadata()) {
            return;
        }
        let target = record.target();
        let (src, channel) = parse_target(target);
        let level = match record.level() {
            log::Level::Error => LogLevel::Error,
            log::Level::Warn => LogLevel::Warn,
            log::Level::Info => LogLevel::Info,
            log::Level::Debug | log::Level::Trace => LogLevel::Debug,
        };
        let _ = self.tx.send(CapturedLog {
            t: self.start.elapsed().as_secs_f32(),
            src: src.to_string(),
            channel: channel.to_string(),
            level,
            msg: format!("{}", record.args()),
        });
    }
    fn flush(&self) {
        self.inner.flush();
    }
}

/// Map a `log` target ("sim::server::accept") to the HUD's (source,
/// channel) pair.
///
///   sim::server / sim::bridge      → ("sim",      "ipc")
///   sim::*                         → ("sim",      "core")
///   galipeur::strat::*             → ("galipeur", "strat")
///   <crate>::<channel>::*          → matching channel if it's one of
///                                     the 7 known names; else "core"
fn parse_target(target: &str) -> (&'static str, &'static str) {
    let mut parts = target.split("::");
    let first = parts.next().unwrap_or("");
    let second = parts.next().unwrap_or("");

    // Canonical source — must match `LOG_SOURCES` for filter UI.
    let src: &'static str = match first {
        "sim" => "sim",
        "galipeur" => "galipeur",
        "picotter" | "picotter_emu" | "pami" => "pami",
        // Anything else (wgpu, bevy, etc.) is grouped under `sim` — the
        // HUD only surfaces 3 sources.
        _ => "sim",
    };

    // Canonical channel.
    if (first == "sim") && (second == "server" || second == "bridge") {
        return (src, "ipc");
    }
    let channel = if let Some(found) = LOG_CHANNELS.iter().find(|c| **c == second) {
        *found
    } else {
        "core"
    };
    (src, channel)
}

// ─── Bevy resource + drain system ──────────────────────────────────

const HUD_LOG_CAP: usize = 200;

pub fn drain_into_log_state(rx: Res<LogCaptureRx>, mut state: ResMut<HudLogState>) {
    let mut got = false;
    while let Ok(rec) = rx.0.try_recv() {
        state.entries.push(LogEntry {
            t: rec.t,
            src: rec.src,
            channel: rec.channel,
            level: rec.level,
            msg: rec.msg,
        });
        got = true;
    }
    if got && state.entries.len() > HUD_LOG_CAP {
        let drop = state.entries.len() - HUD_LOG_CAP;
        state.entries.drain(..drop);
    }
}
