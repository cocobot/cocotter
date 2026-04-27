//! Strat-level calibration routines.
//!
//! They drive the asserv to sweep the robot through a sequence of
//! known poses and read back sensor values so we can compute
//! per-sensor mounting parameters.
//!
//! These are meant to be called once (from a dedicated match / menu
//! option), not during a real match.

use std::time::Duration;

use asserv::holonomic::RobotSide;
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
    // Settle > 2× the sim's ground-lidar CAN period (250 ms) so the
    // cached reading is guaranteed to come from a fresh frame *after*
    // the rotation finishes — calibration only runs once, accuracy
    // matters more than wall-clock.
    const SETTLE_MS: u64 = 800;
    // Readings close to the lidar's configured `max_range_mm` are the
    // "no hit" sentinel — drop them.
    const MAX_VALID_MM: u16 = 900;
    // IRL the calibration fixture is finite, so beams that miss it
    // can return suspiciously short readings off random surfaces.
    // Anything closer than half `CAL_X_MM` is presumed garbage.
    const MIN_VALID_MM: u16 = (CAL_X_MM as u16) / 2;

    log::info!(
        "[ground-cal] start at ({CAL_X_MM:.0}, {CAL_Y_MM:.0}), sweeping {STEPS} headings"
    );
    sensors.ground_lidar(RobotSide::Back);
    asserv.teleport(CAL_X_MM, CAL_Y_MM, 0.0);

    //lidar warm up
    std::thread::sleep(Duration::from_millis(3000));

    // Per-step: (actual_theta, [reading per lidar index; None = no hit]).
    let mut samples: Vec<(f32, [Option<u16>; 6])> = Vec::with_capacity(STEPS);
    for i in 0..STEPS {
        let theta = (i as f32) * STEP_DEG.to_radians();
        if asserv.goto_a(theta).is_err() {
            log::error!("[ground-cal] goto_a({theta:.3}) failed");
            return;
        }
        std::thread::sleep(Duration::from_millis(SETTLE_MS));
        let modules = sensors.ground_lidar_all();
        let pos = asserv.position();
        let mut row: [Option<u16>; 6] = [None; 6];
        for m in 0..3 {
            for lane in 0..2 {
                let d = if lane == 0 { modules[m].distance_0 } else { modules[m].distance_1 };
                if d >= MIN_VALID_MM && d < MAX_VALID_MM {
                    row[m * 2 + lane] = Some(d);
                }
            }
        }
        log::info!(
            "[ground-cal] theta={theta:.3} rad: readings={:?} real angle={:.3} pos=({:.2}, {:.2})",
            row, pos.a, pos.x, pos.y,
        );
        // Use the *measured* heading rather than the commanded one
        // (small but bias-prone PID residual). x/y are kept logged-only
        // — they come from encoder integration and would taint the LSQ
        // IRL.
        samples.push((pos.a, row));
    }

    log::info!("[ground-cal] sweep done, fitting {} samples", samples.len());

    // CSV dump so we can replay the LSQ offline (Python / notebook) and
    // tune the fit without running the whole sweep again. Empty cell =
    // None reading. Comment line at top so a header parser can skip it.
    println!("# ground-cal samples — theta_rad, r0, r1, r2, r3, r4, r5 (mm; empty = no hit)");
    println!("theta_rad,r0,r1,r2,r3,r4,r5");
    for &(th, row) in &samples {
        let cell = |i: usize| -> String {
            row[i].map_or(String::new(), |d| d.to_string())
        };
        println!(
            "{th:.6},{},{},{},{},{},{}",
            cell(0), cell(1), cell(2), cell(3), cell(4), cell(5),
        );
    }
    println!("# end ground-cal samples");


    // Only keep samples within this angular window around the lidar's
    // observed minimum. Tight enough to drop readings that hit
    // something other than the fixture (IRL clutter — anything that
    // isn't the calibration wall), wide enough to leave plenty of
    // usable points for the LSQ.
    const FIT_WINDOW_DEG: f32 = 30.0;

    // Fit each lidar → (x_mm, y_mm, theta_rad, r_min_mm, theta_min_rad).
    //
    // Model (wall at asserv.x = 0, robot at x = CAL_X):
    //   reading · cos(theta_R + theta_L) =
    //       -(CAL_X + cos(theta_R)·x_L - sin(theta_R)·y_L)
    //
    // Linear in (x_L, y_L) for fixed theta_L. Grid-search theta_L and
    // pick the smallest residual. The parabolic pre-fit only serves
    // to centre the clutter-rejection window — its `theta_min`/`r_min`
    // are *never* used in the (x_L, y_L, theta_L) calculation. The
    // reported `r_min` comes from the fitted model, not the parabola.
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

        // ---- Window centring (parabolic fit, used only to find a
        // rough theta_min) ----
        let raw_min_idx = all_valid
            .iter()
            .enumerate()
            .min_by(|a, b| {
                a.1 .1.partial_cmp(&b.1 .1).unwrap_or(core::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap();
        let (theta_min_raw, _) = all_valid[raw_min_idx];
        let wrap_pi = |x: f32| -> f32 {
            ((x + core::f32::consts::PI).rem_euclid(core::f32::consts::TAU))
                - core::f32::consts::PI
        };
        const PARA_WINDOW_DEG: f32 = 15.0;
        let para_window = PARA_WINDOW_DEG.to_radians();
        let para_pts: Vec<(f32, f32)> = all_valid
            .iter()
            .copied()
            .filter(|&(th, _)| wrap_pi(th - theta_min_raw).abs() <= para_window)
            .collect();
        let theta_min_para = if para_pts.len() >= 3 {
            let mut su = 0.0_f64;
            let mut su2 = 0.0_f64;
            let mut su3 = 0.0_f64;
            let mut su4 = 0.0_f64;
            let mut sr = 0.0_f64;
            let mut sur = 0.0_f64;
            let mut su2r = 0.0_f64;
            let n = para_pts.len() as f64;
            for &(th, r) in &para_pts {
                let u = wrap_pi(th - theta_min_raw) as f64;
                let r = r as f64;
                let u2 = u * u;
                su += u;
                su2 += u2;
                su3 += u2 * u;
                su4 += u2 * u2;
                sr += r;
                sur += u * r;
                su2r += u2 * r;
            }
            let det = su4 * (su2 * n - su * su)
                - su3 * (su3 * n - su * su2)
                + su2 * (su3 * su - su2 * su2);
            if det.abs() > 1e-12 {
                let det_a = su2r * (su2 * n - su * su)
                    - su3 * (sur * n - su * sr)
                    + su2 * (sur * su - su2 * sr);
                let det_b = su4 * (sur * n - su * sr)
                    - su2r * (su3 * n - su * su2)
                    + su2 * (su3 * sr - sur * su2);
                let a = det_a / det;
                let b = det_b / det;
                if a.abs() > 1e-12 {
                    let u = -b / (2.0 * a);
                    let para_window_f64 = para_window as f64;
                    if u.is_finite() && u.abs() <= para_window_f64 {
                        (theta_min_raw as f64 + u) as f32
                    } else {
                        theta_min_raw
                    }
                } else {
                    theta_min_raw
                }
            } else {
                theta_min_raw
            }
        } else {
            theta_min_raw
        };

        // ---- Apply window filter (clutter rejection) ----
        let window = FIT_WINDOW_DEG.to_radians();
        let valid: Vec<(f32, f32)> = all_valid
            .iter()
            .copied()
            .filter(|&(th, _)| {
                let mut d = th - theta_min_para;
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

        // ---- Physical model LSQ for (x_L, y_L) at each candidate theta_L ----
        let solve_lsq = |theta_l: f64| -> Option<(f64, f64, f64)> {
            let (mut sxx, mut sxy, mut syy, mut sxc, mut syc) =
                (0.0_f64, 0.0, 0.0, 0.0, 0.0);
            let cal_x = CAL_X_MM as f64;
            for &(th, r) in &valid {
                let th = th as f64;
                let r = r as f64;
                let a_i = -th.cos();
                let b_i = th.sin();
                let c_i = r * (th + theta_l).cos() + cal_x;
                sxx += a_i * a_i;
                sxy += a_i * b_i;
                syy += b_i * b_i;
                sxc += a_i * c_i;
                syc += b_i * c_i;
            }
            let det = sxx * syy - sxy * sxy;
            if det.abs() < 1e-9 {
                return None;
            }
            let x_l = (sxc * syy - syc * sxy) / det;
            let y_l = (syc * sxx - sxc * sxy) / det;
            let mut rss = 0.0_f64;
            for &(th, r) in &valid {
                let th = th as f64;
                let r = r as f64;
                let a_i = -th.cos();
                let b_i = th.sin();
                let c_i = r * (th + theta_l).cos() + cal_x;
                let res = a_i * x_l + b_i * y_l - c_i;
                rss += res * res;
            }
            Some((x_l, y_l, rss))
        };

        let mut best: Option<(f64, f64, f64, f64)> = None; // (rss, x, y, theta_l)
        for i in 0..360 {
            let theta_l = (i as f64).to_radians();
            if let Some((x, y, rss)) = solve_lsq(theta_l) {
                if best.map_or(true, |b| rss < b.0) {
                    best = Some((rss, x, y, theta_l));
                }
            }
        }
        if let Some((_, _, _, t0)) = best {
            let mut best_fine = best;
            let mut d = -1.0_f64;
            while d <= 1.0 {
                let theta_l = t0 + d.to_radians();
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

        // r_min / theta_min reported come from the *fitted model*, not
        // from the parabolic fit on integer samples. Sub-mm precision.
        let cal_x = CAL_X_MM as f64;
        let mut r_min_f64 = f64::INFINITY;
        let mut theta_min_f64 = 0.0_f64;
        for i in 0..3600 {
            let th = (i as f64) * 0.1_f64.to_radians();
            let denom = (th + theta_l).cos();
            if denom.abs() < 1e-3 {
                continue;
            }
            let r = -(cal_x + th.cos() * x_l - th.sin() * y_l) / denom;
            if r > 0.0 && r < r_min_f64 {
                r_min_f64 = r;
                theta_min_f64 = th;
            }
        }

        let theta_l = (theta_l + core::f64::consts::PI)
            .rem_euclid(core::f64::consts::TAU)
            - core::f64::consts::PI;
        fits[lidar] = Some((
            x_l as f32,
            y_l as f32,
            theta_l as f32,
            r_min_f64 as f32,
            theta_min_f64 as f32,
        ));
    }

    // Render a ready-to-paste `GroundLidarConf` for `galipeur/src/main.rs`.
    // The CAN wire layout packs lidars as module[i/2][i%2], so calib
    // index 2*m + l maps directly to `modules[m][l]`.
    println!("// Paste into galipeur/src/main.rs (GroundLidarConf):");
    println!("GroundLidarConf {{");
    println!("    modules: [");
    for m in 0..3 {
        println!("        // Module {m}: lidar {} and lidar {}", m, m + 3);
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
