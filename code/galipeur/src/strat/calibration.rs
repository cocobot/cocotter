//! Ground-lidar sample sweep.
//!
//! Galipeur only acquires raw samples and dumps them as CSV via
//! `println!`. The actual fit (per-lidar `(x, y, theta)` in the body
//! frame) lives in `galipeur/calib/analyze.py` so we can iterate on
//! the LSQ without re-flashing or re-running the sweep.
//!
//! Procedure
//! =========
//! 1. Place the robot at `(CAL_X_MM, CAL_Y_MM, 0)` with a flat wall at
//!    `asserv.x = 0` (positive `CAL_X_MM` puts the robot in front of
//!    the wall, heading 0 = away from it).
//! 2. Forward sweep: 1.5 turns, 5° step — fast pass to cover all 6
//!    lidars at every relative heading.
//! 3. Reverse sweep: 1.5 turns the other way, same 5° step but several
//!    samples averaged at each step — slower so the wall is observed
//!    multiple times per heading and noise is averaged out.
//!
//! Output marker lines (`# ground-cal …`) bracket the CSV so the
//! Python script can ignore other log spam.

use std::time::Duration;

use asserv::holonomic::RobotSide;
use board_sabotter::SabotterBoard;

use crate::sensors::{GroundLidarModule, Sensors, TopLidarSnapshot};
use crate::strat::utils::AsservHelper;

/// Robot pose for the sweep. Wall at `asserv.x = 0`, robot at
/// `(CAL_X_MM, 0)` heading 0 — wall lies behind the robot. Keep
/// `CAL_X_MM` within the lidar `max_range` (~1 m) and big enough that
/// the chassis doesn't touch the wall while rotating.
pub const CAL_X_MM: f32 = 300.0;
pub const CAL_Y_MM: f32 = 0.0;

/// 5° step, 1.5 turns each direction.
const STEP_DEG: f32 = 5.0;
const TURNS: f32 = 1.5;
const STEPS_PER_PASS: usize = (360.0 * TURNS / STEP_DEG) as usize;

/// Lidar measurements need ~4× the CAN period to fully settle (motion
/// blur on the spinning head + low-pass filtering inside the module).
const SETTLE_MS_FAST: u64 = 1600;
const SETTLE_MS_SLOW: u64 = 2400;
/// Reverse pass takes several samples per stop with this spacing.
const SAMPLES_PER_STEP_SLOW: usize = 5;
const SAMPLE_DT_MS: u64 = 80;

#[allow(unused)]
pub fn ground_lidars_sample<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
) {
    log::info!(
        "[ground-cal] start at ({CAL_X_MM:.0}, {CAL_Y_MM:.0}), \
         {STEPS_PER_PASS} steps × {STEP_DEG}° per pass"
    );

    // Power-on the ground lidars (side picked arbitrarily — the call
    // sends the same `SetLidarEnable` regardless).
    sensors.ground_lidar(RobotSide::Back);
    asserv.teleport(CAL_X_MM, CAL_Y_MM, 0.0);

    // Lidar warm-up: first few revolutions are noisy.
    std::thread::sleep(Duration::from_secs(3));

    // CSV header. The Python analyzer skips '#'-prefixed lines.
    // Ground-cal and top-cal data are interleaved but distinguished
    // by their column count (14 vs 6) and parsed independently.
    println!("# ground-cal samples — BEGIN");
    println!("# columns: pass = fwd|rev, theta_rad = measured asserv.a, dN/sqN per lidar");
    println!("pass,theta_rad,d0,sq0,d1,sq1,d2,sq2,d3,sq3,d4,sq4,d5,sq5");
    println!("# top-cal: pass,theta_rad,step_idx,ld06_angle_deg,ld06_dist_mm,ld06_intensity");

    let mut step_idx: u32 = 0;

    // ---- Pass 1: forward, 5° step, 1.5 turns (clockwise) ----
    log::info!("[ground-cal] forward sweep");
    for i in 0..=STEPS_PER_PASS {
        let theta = -(i as f32) * STEP_DEG.to_radians();
        if asserv.goto_a(theta).is_err() {
            log::error!("[ground-cal] goto_a({theta:.3}) failed");
            return;
        }
        std::thread::sleep(Duration::from_millis(SETTLE_MS_FAST));
        let mods = sensors.ground_lidar_all();
        let pos = asserv.position();
        print_sample("fwd", pos.a, &mods);
        let scan = sensors.top_lidar_scan();
        print_top_scan("fwd", pos.a, step_idx, &scan);
        // Extra delay to let the UART drain the LD06 scan line.
        std::thread::sleep(Duration::from_secs(1));
        step_idx += 1;
    }

    // ---- Pass 2: reverse, 5° step, 1.5 turns (counter-clockwise) ----
    log::info!("[ground-cal] reverse sweep (slow)");
    let start_theta = -(STEPS_PER_PASS as f32) * STEP_DEG.to_radians();
    for i in 0..=STEPS_PER_PASS {
        let theta = start_theta + (i as f32) * STEP_DEG.to_radians();
        if asserv.goto_a(theta).is_err() {
            log::error!("[ground-cal] goto_a({theta:.3}) failed");
            return;
        }
        std::thread::sleep(Duration::from_millis(SETTLE_MS_SLOW));
        for j in 0..SAMPLES_PER_STEP_SLOW {
            let mods = sensors.ground_lidar_all();
            let pos = asserv.position();
            print_sample("rev", pos.a, &mods);
            // Dump one LD06 scan per stop (first sub-sample only to
            // keep output size manageable — the LD06 doesn't change
            // meaningfully within 5×80 ms).
            if j == 0 {
                let scan = sensors.top_lidar_scan();
                print_top_scan("rev", pos.a, step_idx, &scan);
                std::thread::sleep(Duration::from_secs(1));
            }
            std::thread::sleep(Duration::from_millis(SAMPLE_DT_MS));
        }
        step_idx += 1;
    }

    println!("# ground-cal samples — END");
    log::info!("[ground-cal] done");
}

fn print_top_scan(pass: &str, theta: f32, step_idx: u32, scan: &TopLidarSnapshot) {
    // Pack the entire scan into one CSV line to avoid stdout buffer
    // overflow on ESP32.  Format: pass,theta,step_idx,a1:d1;a2:d2;...
    // (angle_deg:dist_mm pairs, semicolon-separated, skip dist==0).
    use core::fmt::Write;
    let mut buf = String::with_capacity(4096);
    let _ = write!(buf, "{pass},{theta:.6},{step_idx}");
    for &(angle_deg, dist_mm, _intensity) in &scan.points {
        if dist_mm > 0 {
            let _ = write!(buf, ",{angle_deg:.1}:{dist_mm}");
        }
    }
    println!("{buf}");
}

fn print_sample(pass: &str, theta: f32, mods: &[GroundLidarModule; 3]) {
    println!(
        "{pass},{theta:.6},{},{},{},{},{},{},{},{},{},{},{},{}",
        mods[0].distance_0, mods[0].sq_0,
        mods[0].distance_1, mods[0].sq_1,
        mods[1].distance_0, mods[1].sq_0,
        mods[1].distance_1, mods[1].sq_1,
        mods[2].distance_0, mods[2].sq_0,
        mods[2].distance_1, mods[2].sq_1,
    );
}
