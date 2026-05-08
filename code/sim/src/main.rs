//! Simulator for meca_cocotter robots.
//!
//! Robots (galipeur, pami) connect via Unix socket. The sim owns the world,
//! replies with ground-truth feedback, and renders everything in a Bevy 3D
//! scene (or runs headless in CI).

use clap::Parser;
use sim_protocol::{RobotKind, Side, SimMsgS2C};

mod app;
mod bridge;
mod collide;
mod config;
mod controls;
mod hud;
mod humans;
mod joints;
mod kinematics;
mod ld06_encoder;
mod picotter_emu;
mod raycast;
mod server;
mod textures;
mod world;

#[derive(Parser, Debug)]
#[command(name = "sim", about = "Robotics simulator for galipeur + pami")]
struct Cli {
    /// Run without the 3D rendering window (useful for CI).
    #[arg(long)]
    headless: bool,

    /// Unix socket path to listen on.
    #[arg(long, default_value = sim_protocol::DEFAULT_SOCKET_PATH)]
    socket: String,

    /// Field configuration TOML file.
    #[arg(long, default_value = "sim/cfg/table.toml")]
    field: String,

    /// Random seed (for deterministic tests).
    #[arg(long)]
    seed: Option<u64>,

    /// Simulation speed factor (1.0 = realtime, 5.0 = fast-forward).
    #[arg(long, default_value_t = 1.0)]
    time_factor: f32,

    /// Spawn one or more robots at startup. Format: `KIND:SIDE` for
    /// galipeur/pami (SIDE ∈ {left, right}), or bare `adversary` for
    /// the in-process opponent (no side). Repeatable.
    /// Example: `--spawn galipeur:left --spawn pami:right --spawn adversary`.
    #[arg(long = "spawn", value_name = "KIND[:SIDE]")]
    spawn: Vec<String>,

    /// Once every spawned robot is connected, automatically run the
    /// starter sequence (insert → wait → remove). Useful for headless
    /// integration tests. The two delays are configurable below.
    #[arg(long)]
    auto_start: bool,

    /// Seconds to wait for all `--spawn` robots to register before
    /// firing the first T (starter inserted).
    #[arg(long, default_value_t = 5.0)]
    auto_start_connect_timeout_s: f32,

    /// Seconds the simulated starter cable stays inserted before the
    /// auto-start fires the second T (starter removed → match runs).
    #[arg(long, default_value_t = 3.0)]
    auto_start_setup_s: f32,
}

#[derive(Clone, Copy, Debug)]
enum SpawnSpec {
    Robot(RobotKind, Side),
    Adversary,
}

fn parse_spawn(spec: &str) -> Result<SpawnSpec, String> {
    if spec == "adversary" {
        return Ok(SpawnSpec::Adversary);
    }
    let (kind_s, side_s) = spec
        .split_once(':')
        .ok_or_else(|| format!("expected KIND:SIDE or 'adversary', got {spec:?}"))?;
    let kind = match kind_s {
        "galipeur" => RobotKind::Galipeur,
        "pami" => RobotKind::Pami,
        "adversary" => {
            return Err(format!(
                "adversary doesn't take a side, use bare 'adversary' (got {spec:?})"
            ))
        }
        other => return Err(format!("unknown kind {other:?} in {spec:?}")),
    };
    let side = match side_s {
        "left" => Side::Left,
        "right" => Side::Right,
        other => return Err(format!("unknown side {other:?} in {spec:?}")),
    };
    Ok(SpawnSpec::Robot(kind, side))
}

fn main() {
    // HUD log capture + env_logger stderr in one go (Phase 10d). The
    // returned Receiver is plumbed into the Bevy app below.
    let log_rx = app::install_logger(
        "info,wgpu=error,naga=warn,wgpu_hal=warn,wgpu_core=warn,bevy_mod_outline::queue=off",
    );

    // wgpu backend selection knobs (set before any wgpu init):
    //   WGPU_BACKEND=vulkan|gl|dx12|metal   — pick a specific backend
    //   WGPU_FORCE_FALLBACK_ADAPTER=1       — llvmpipe CPU renderer (slow but stable)
    //   WGPU_POWER_PREFERENCE=low|high      — adapter selection
    //
    // On some amdgpu driver/firmware versions (notably Radeon 780M with
    // kernels < 6.7 missing the newer SMU interface), Vulkan init can
    // crash the GPU. If you hit that, try in order:
    //   WGPU_FORCE_FALLBACK_ADAPTER=1 cargo run -p sim
    //   WGPU_BACKEND=gl cargo run -p sim   (needs mesa-libEGL)

    let cli = Cli::parse();

    log::info!(
        "sim starting: headless={} socket={} field={} time_factor={}",
        cli.headless,
        cli.socket,
        cli.field,
        cli.time_factor,
    );

    let config = match config::Config::load(&cli.field) {
        Ok(c) => c,
        Err(e) => {
            log::error!("failed to load {}: {e}", cli.field);
            std::process::exit(1);
        }
    };

    // Validate spawn specs early so a typo doesn't slip past the
    // listener-bring-up.
    let initial_spawns: Vec<SpawnSpec> = cli
        .spawn
        .iter()
        .map(|s| {
            parse_spawn(s).unwrap_or_else(|e| {
                log::error!("--spawn {s:?}: {e}");
                std::process::exit(1);
            })
        })
        .collect();

    let (tx, rx) = bridge::channel();
    let shared_world = world::World::new();
    let conn_registry = controls::ConnRegistry::default();
    let match_state = controls::MatchStateLock::default();

    // Single source of truth for the live config. Bevy uses its own
    // `SimConfigRes` clone for read access (no lock contention) but the
    // R-reset handler also writes through this Arc<RwLock<>> so that
    // freshly-arriving Hello messages, served on the listener thread,
    // pick up the reloaded values without a sim restart.
    use std::sync::{Arc, RwLock};
    let shared_config: Arc<RwLock<config::Config>> = Arc::new(RwLock::new(config.clone()));

    // IPC listener runs on its own thread. Each accepted connection gets a
    // spawned sub-thread inside listen_forever.
    let socket = cli.socket.clone();
    let server_config = Arc::clone(&shared_config);
    let world_for_server = shared_world.clone();
    let tx_for_server = tx.clone();
    let registry_for_server = conn_registry.clone();
    let match_state_for_server = match_state.clone();
    std::thread::Builder::new()
        .name("sim-listener".into())
        .spawn(move || {
            if let Err(e) = server::listen_forever(
                &socket,
                server_config,
                tx_for_server,
                world_for_server,
                registry_for_server,
                match_state_for_server,
            ) {
                log::error!("sim server error: {e}");
                std::process::exit(1);
            }
        })
        .expect("spawn sim-listener");

    // Launch CLI-requested robots. Galipeur/pami connect to the listener
    // once it's up; the SpawnSlots resource is pre-seeded so the chord's
    // `R` reset will relaunch the same set. Adversary is in-process and
    // gets materialized inside Bevy via `spawn_initial_adversary`.
    let mut initial_slots = controls::SpawnSlots::default();
    for spec in &initial_spawns {
        match *spec {
            SpawnSpec::Robot(kind, side) => {
                controls::launch_robot(kind, side, &config.teams);
                initial_slots.set(kind, side, true);
            }
            SpawnSpec::Adversary => {
                initial_slots.request_adversary_spawn = true;
            }
        }
    }

    // Headless auto-start: simulate the T-T sequence after delays.
    // Runs on a side thread so we don't block the listener.
    if cli.auto_start {
        let conns = conn_registry.clone();
        let ms = match_state.clone();
        let connect_timeout = cli.auto_start_connect_timeout_s;
        let setup_s = cli.auto_start_setup_s;
        // Adversary doesn't connect via IPC, so it shouldn't count
        // against the auto-start "wait for connections" budget.
        let expected = initial_spawns
            .iter()
            .filter(|s| matches!(s, SpawnSpec::Robot(..)))
            .count();
        std::thread::Builder::new()
            .name("auto-start".into())
            .spawn(move || run_auto_start(conns, ms, expected, connect_timeout, setup_s))
            .expect("spawn auto-start");
    }

    // Random humans walking around the table (no-op when disabled).
    let tx_for_app = tx.clone();
    humans::spawn(
        config.humans.clone(),
        config.field.clone(),
        tx,
        shared_world.clone(),
    );

    // Bevy takes over the main thread.
    app::run(
        config,
        cli.field.clone(),
        shared_config,
        rx,
        cli.headless,
        conn_registry,
        match_state,
        initial_slots,
        tx_for_app,
        shared_world,
        log_rx,
    );
}

/// Wait for `expected` robots to register (or `timeout_s` to elapse),
/// then T (insert), wait `setup_s`, T (remove). Used by `--auto-start`
/// for headless / CI runs.
fn run_auto_start(
    conns: controls::ConnRegistry,
    match_state: controls::MatchStateLock,
    expected: usize,
    connect_timeout_s: f32,
    setup_s: f32,
) {
    use std::time::{Duration, Instant};
    let deadline = Instant::now() + Duration::from_secs_f32(connect_timeout_s);
    while Instant::now() < deadline {
        if conns.0.lock().unwrap().len() >= expected.max(1) {
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let connected = conns.0.lock().unwrap().len();
    log::info!(
        "[auto-start] {connected}/{expected} robot(s) connected, inserting starter"
    );
    conns.broadcast(&SimMsgS2C::Starter { inserted: true });
    match_state.set(controls::MatchState::StarterInserted);

    std::thread::sleep(Duration::from_secs_f32(setup_s));
    log::info!("[auto-start] removing starter — match running");
    conns.broadcast(&SimMsgS2C::Starter { inserted: false });
    match_state.set(controls::MatchState::Running);
}
