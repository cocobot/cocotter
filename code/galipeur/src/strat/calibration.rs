//! Strat-level calibration routines.
//!
//! They drive the asserv to sweep the robot through a sequence of
//! known poses and read back sensor values so we can compute
//! per-sensor mounting parameters.
//!
//! These are meant to be called once (from a dedicated match / menu
//! option), not during a real match.

use std::time::Duration;

use board_sabotter::SabotterBoard;

use crate::sensors::Sensors;
use crate::strat::utils::AsservHelper;

/// Starting pose for the ground-lidar sweep. IRL the operator wedges
/// the robot against a fixture at this (x, y) with heading 0; in the
/// simulator `asserv.teleport` does it automatically.
///
/// Keep `CAL_Y` small enough that every lidar can actually reach the
/// wall (beam `max_range`, ~1 m), big enough to avoid the chassis
/// touching the wall as the robot rotates.
/// The calibration expects a reference wall at `asserv.x = 0`. The
/// robot is placed at `asserv.x = CAL_X_MM` (positive), so the wall
/// lies behind the robot at heading 0, and the lidars will sweep
/// through its perpendicular as the robot rotates. IRL you wedge the
/// chassis against a flat surface; in the sim the user's
/// `MovementLowLevelHardware::teleport` maps this onto sim's north
/// wall (sim.y=2000) via `sim.y = 2000 - asserv.x`.
pub const CAL_X_MM: f32 = 300.0;
pub const CAL_Y_MM: f32 = 0.0;

/// Full 360° sweep calibration of the 6 ground lidars.
///
/// Procedure
/// =========
/// 1. Put the robot at `(CAL_X_MM, CAL_Y_MM, 0)` — sim teleports, IRL
///    the operator wedges the chassis and triggers the routine.
/// 2. Rotate through `STEPS` headings (5° step) via `asserv.goto_a`;
///    both sim and real hardware drive motors here, so the mechanics
///    are identical.
/// 3. At each heading, wait `SETTLE_MS` (> lidar CAN period) then
///    grab the 6 distance readings.
/// 4. Fit each lidar's `(x, y, theta)` in the body frame.
///
/// Fit
/// ===
/// Wall at `asserv.x = 0`, robot at `(CAL_X_MM, 0)`. The min reading
/// for a lidar happens when its beam is perpendicular to the wall,
/// i.e. pointing along world `-X_asserv` (angle `pi`):
/// `theta_R + theta_L = pi  →  theta_L = pi - theta_R_min`.
///
/// With `theta_L` fixed, every valid sample yields a linear equation:
///
/// ```text
/// reading · cos(theta_R + theta_L) = -(CAL_X + cos(theta_R)·x_L - sin(theta_R)·y_L)
/// ```
///
/// Stacking samples gives a 2×2 normal-equation system that's solved
/// analytically. No `z_L` is recovered — ground lidars share the
/// same mounting height, fill it in by hand from the CAD.
pub fn ground_lidars<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
) {
    const STEP_DEG: f32 = 5.0;
    const STEPS: usize = (360.0 / STEP_DEG) as usize;
    // Settle > the sim's ground-lidar CAN period (250 ms).
    const SETTLE_MS: u64 = 400;
    // Readings close to the lidar's configured `max_range_mm` are the
    // "no hit" sentinel — drop them.
    const MAX_VALID_MM: u16 = 900;
    // IRL the calibration fixture is finite, so beams that miss it
    // can return suspiciously short readings off random surfaces.
    // Anything closer than half `CAL_X_MM` is presumed garbage.
    const MIN_VALID_MM: u16 = (CAL_X_MM as u16) / 2;
    // Only keep samples within this angular window around the lidar's
    // observed minimum. Tight enough to drop readings that hit
    // something other than the fixture, wide enough to leave plenty
    // of usable points for the LSQ.
    const FIT_WINDOW_DEG: f32 = 30.0;

    log::info!(
        "[ground-cal] start at ({CAL_X_MM:.0}, {CAL_Y_MM:.0}), sweeping {STEPS} headings"
    );
    asserv.teleport(CAL_X_MM, CAL_Y_MM, 0.0);

    // Per-step: (theta_R, [reading per lidar index; None = no hit])
    let mut samples: Vec<(f32, [Option<u16>; 6])> = Vec::with_capacity(STEPS);
    for i in 0..STEPS {
        let theta = (i as f32) * STEP_DEG.to_radians();
        if asserv.goto_a(theta).is_err() {
            log::error!("[ground-cal] goto_a({theta:.3}) failed");
            return;
        }
        std::thread::sleep(Duration::from_millis(SETTLE_MS));
        let modules = sensors.ground_lidar_all();
        let mut row: [Option<u16>; 6] = [None; 6];
        for m in 0..3 {
            for lane in 0..2 {
                let d = if lane == 0 { modules[m].distance_0 } else { modules[m].distance_1 };
                if d >= MIN_VALID_MM && d < MAX_VALID_MM {
                    row[m * 2 + lane] = Some(d);
                }
            }
        }
        samples.push((theta, row));
    }

    log::info!("[ground-cal] sweep done, fitting {} samples", samples.len());

    // Fit each lidar → (x_mm, y_mm, theta_rad, r_min_mm, theta_min_rad)
    let mut fits: [Option<(f32, f32, f32, f32, f32)>; 6] = [None; 6];
    for lidar in 0..6 {
        let all_valid: Vec<(f32, f32)> = samples
            .iter()
            .filter_map(|(th, row)| row[lidar].map(|d| (*th, d as f32)))
            .collect();
        if all_valid.len() < 5 {
            log::warn!(
                "[ground-cal] lidar {lidar}: only {} valid readings, skipping",
                all_valid.len()
            );
            continue;
        }
        let (theta_min, r_min) =
            all_valid.iter().copied().fold((0.0_f32, f32::INFINITY), |acc, (th, r)| {
                if r < acc.1 { (th, r) } else { acc }
            });
        // IRL the fixture is narrow; readings outside a window around
        // the angle of the global minimum tend to come from background
        // clutter rather than the calibration wall. Drop them.
        let window = FIT_WINDOW_DEG.to_radians();
        let valid: Vec<(f32, f32)> = all_valid
            .iter()
            .copied()
            .filter(|&(th, _)| {
                let mut d = th - theta_min;
                d = d.rem_euclid(core::f32::consts::TAU);
                if d > core::f32::consts::PI {
                    d -= core::f32::consts::TAU;
                }
                d.abs() <= window
            })
            .collect();
        if valid.len() < 5 {
            log::warn!(
                "[ground-cal] lidar {lidar}: only {} samples in ±{FIT_WINDOW_DEG}° \
                 window, skipping",
                valid.len()
            );
            continue;
        }

        // Grid search over theta_L. For a non-radial lidar, the min
        // reading isn't exactly at the perpendicular heading, so
        // argmin(reading) doesn't recover theta_L directly — hence the
        // search over 1° candidates, picking the one with the smallest
        // LSQ residual on (x_L, y_L).
        //
        // Model (wall at asserv.x = 0, robot at x = CAL_X):
        //   reading · cos(theta_R + theta_L) =
        //       -(CAL_X + cos(theta_R)·x_L - sin(theta_R)·y_L)
        //
        // Rearranged to be linear in (x_L, y_L) with coefficients
        // (-cos θ_R, sin θ_R) and RHS (reading · cos(θ_R + θ_L) + CAL_X).
        let solve_lsq = |theta_l: f32| -> Option<(f32, f32, f32)> {
            let (mut sxx, mut sxy, mut syy, mut sxc, mut syc) =
                (0.0_f32, 0.0, 0.0, 0.0, 0.0);
            for &(th, r) in &valid {
                let a_i = -th.cos();
                let b_i = th.sin();
                let c_i = r * (th + theta_l).cos() + CAL_X_MM;
                sxx += a_i * a_i;
                sxy += a_i * b_i;
                syy += b_i * b_i;
                sxc += a_i * c_i;
                syc += b_i * c_i;
            }
            let det = sxx * syy - sxy * sxy;
            if det.abs() < 1e-6 {
                return None;
            }
            let x_l = (sxc * syy - syc * sxy) / det;
            let y_l = (syc * sxx - sxc * sxy) / det;
            let mut rss = 0.0_f32;
            for &(th, r) in &valid {
                let a_i = -th.cos();
                let b_i = th.sin();
                let c_i = r * (th + theta_l).cos() + CAL_X_MM;
                let res = a_i * x_l + b_i * y_l - c_i;
                rss += res * res;
            }
            Some((x_l, y_l, rss))
        };

        // Coarse pass: every 1°.
        let mut best: Option<(f32, f32, f32, f32)> = None; // (rss, x, y, theta_l)
        for i in 0..360 {
            let theta_l = (i as f32).to_radians();
            if let Some((x, y, rss)) = solve_lsq(theta_l) {
                if best.map_or(true, |b| rss < b.0) {
                    best = Some((rss, x, y, theta_l));
                }
            }
        }
        // Fine pass: ±1° around the best candidate, in 0.05° steps.
        if let Some((_, _, _, t0)) = best {
            let mut best_fine = best;
            let mut d = -1.0_f32;
            while d <= 1.0 {
                let theta_l = t0 + (d as f32).to_radians();
                if let Some((x, y, rss)) = solve_lsq(theta_l) {
                    if best_fine.map_or(true, |b| rss < b.0) {
                        best_fine = Some((rss, x, y, theta_l));
                    }
                }
                d += 0.05;
            }
            best = best_fine;
        }
        let Some((_, x_l, y_l, theta_l)) = best else {
            log::warn!("[ground-cal] lidar {lidar}: LSQ search failed");
            continue;
        };
        // Wrap to [-pi, pi] for nicer output.
        let theta_l = ((theta_l + core::f32::consts::PI)
            .rem_euclid(core::f32::consts::TAU))
            - core::f32::consts::PI;
        fits[lidar] = Some((x_l, y_l, theta_l, r_min, theta_min));
    }

    // Render a ready-to-paste `GroundLidarConf` for `galipeur/src/main.rs`.
    // The CAN wire layout packs lidars as module[i/2][i%2], so calib
    // index 2*m + l maps directly to `modules[m][l]`.
    println!("// Paste into galipeur/src/main.rs (GroundLidarConf):");
    println!("GroundLidarConf {{");
    println!("    modules: [");
    for m in 0..3 {
        println!("        // Module {m}: lidar {} and lidar {}", m * 2, m * 2 + 1);
        println!("        [");
        for l in 0..2 {
            let idx = m * 2 + l;
            match fits[idx] {
                Some((x, y, t, r_min, th_min)) => {
                    let deg = t.to_degrees();
                    println!(
                        "            GroundLidarPose {{ x: {x:.2}, y: {y:.2}, \
                         theta: {deg:.3}_f32.to_radians() }},  // r_min={r_min:.0} mm \
                         @ θ_R={:.1}°",
                        th_min.to_degrees()
                    );
                }
                None => {
                    println!(
                        "            GroundLidarPose {{ x: 0.0, y: 0.0, theta: 0.0 }},  // \
                         lidar {idx}: FIT FAILED"
                    );
                }
            }
        }
        println!("        ],");
    }
    println!("    ],");
    println!("}}");
}
