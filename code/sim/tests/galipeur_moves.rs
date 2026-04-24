//! End-to-end integration test: spawn the sim binary, connect as a fake
//! galipeur, command pure forward motion, assert the returned encoder deltas
//! integrate into a positive body-frame displacement.
//!
//! The test does not rely on the galipeur binary — it plays the role of a
//! minimal client against `sim_client`, which exercises the full protocol
//! round-trip.

use std::env;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use sim_client::SimClient;
use sim_protocol::{Capabilities, RobotKind, SimMsgC2S, PROTOCOL_VERSION};

struct SimProcess(Child);

impl Drop for SimProcess {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn find_sim_binary() -> PathBuf {
    // `cargo test` builds test targets; the sim binary lives in the same
    // target dir. Use CARGO_BIN_EXE_sim if present (Cargo sets this for
    // tests in the same package).
    if let Some(p) = option_env!("CARGO_BIN_EXE_sim") {
        return PathBuf::from(p);
    }
    // Fallback: look next to the current test binary.
    let mut dir = env::current_exe().unwrap();
    dir.pop(); // deps
    dir.pop(); // debug
    dir.push("sim");
    dir
}

fn find_field_toml() -> PathBuf {
    // Manifest dir points to sim/.
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("cfg");
    p.push("table.toml");
    p
}

fn spawn_sim(socket: &str) -> SimProcess {
    let bin = find_sim_binary();
    let field = find_field_toml();
    let child = Command::new(&bin)
        .args([
            "--headless",
            "--socket",
            socket,
            "--field",
            field.to_str().unwrap(),
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn sim");
    SimProcess(child)
}

fn wait_for_socket(path: &str, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if std::path::Path::new(path).exists() {
            return;
        }
        thread::sleep(Duration::from_millis(50));
    }
    panic!("sim socket {path} did not appear within {timeout:?}");
}

#[test]
fn galipeur_receives_nonzero_encoder_deltas_for_forward_consigns() {
    let socket = format!("/tmp/meca_sim_test_{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket);
    let _sim = spawn_sim(&socket);
    wait_for_socket(&socket, Duration::from_secs(5));

    let hello = SimMsgC2S::Hello {
        version: PROTOCOL_VERSION,
        robot_id: "galipeur-test".into(),
        kind: RobotKind::Galipeur,
        caps: Capabilities::HOLO3 | Capabilities::CAN,
        requested_start: None,
    };
    let client = SimClient::connect(&socket, hello).expect("connect");

    // Consigns that, under the galipeur motors matrix, correspond roughly to
    // pure +vx body motion (the numbers come from the matrix in the config):
    // row 0 of velocities_to_consigns at vx=100 gives ~13.72, etc. We use
    // PWM-scaled values comfortably below the ±4095 saturation.
    let vx = 200.0; // mm/s
    let consigns = [
        0.137193775559 * vx,
        -0.267514745628 * vx,
        0.138273262887 * vx,
    ];

    let mut total_enc = [0.0f64; 3];
    let mut total_gyro = 0.0f64;
    for _ in 0..100 {
        let (enc, gyro) = client.tick_holo(consigns);
        for i in 0..3 {
            total_enc[i] += enc[i] as f64;
        }
        total_gyro += gyro as f64;
    }

    // Integrated over 100 ticks of 10 ms = 1 s of simulated time, we expect
    // a non-zero total encoder displacement and a near-zero gyro rotation.
    let norm = (total_enc[0] * total_enc[0]
        + total_enc[1] * total_enc[1]
        + total_enc[2] * total_enc[2])
        .sqrt();
    assert!(
        norm > 1.0,
        "expected non-zero encoder deltas, got {:?} (norm {norm:.3})",
        total_enc
    );
    assert!(
        total_gyro.abs() < 0.05,
        "expected near-zero gyro rotation for pure translation, got {total_gyro}"
    );
}
