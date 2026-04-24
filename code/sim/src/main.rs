//! Simulator for meca_cocotter robots.
//!
//! Robots (galipeur, pami) connect via Unix socket. The sim owns the world,
//! replies with ground-truth feedback, and renders everything in a Bevy 3D
//! scene (or runs headless in CI).

use clap::Parser;

mod app;
mod bridge;
mod config;
mod humans;
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

    /// Enable the random humans walking around the table (off by default).
    #[arg(long)]
    humans: bool,
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

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

    let mut config = match config::Config::load(&cli.field) {
        Ok(c) => c,
        Err(e) => {
            log::error!("failed to load {}: {e}", cli.field);
            std::process::exit(1);
        }
    };
    if cli.humans {
        config.humans.enabled = true;
    }

    let (tx, rx) = bridge::channel();
    let shared_world = world::World::new();

    // IPC listener runs on its own thread. Each accepted connection gets a
    // spawned sub-thread inside listen_forever.
    let socket = cli.socket.clone();
    let server_config = config.clone();
    let world_for_server = shared_world.clone();
    let tx_for_server = tx.clone();
    std::thread::Builder::new()
        .name("sim-listener".into())
        .spawn(move || {
            if let Err(e) = server::listen_forever(&socket, server_config, tx_for_server, world_for_server) {
                log::error!("sim server error: {e}");
                std::process::exit(1);
            }
        })
        .expect("spawn sim-listener");

    // Random humans walking around the table (no-op when disabled).
    humans::spawn(
        config.humans.clone(),
        config.field.clone(),
        tx,
        shared_world.clone(),
    );

    // Bevy takes over the main thread.
    app::run(config, rx, cli.headless);
}
