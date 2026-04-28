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

use crate::meca::RobotSideModule;
use crate::sensors::Sensors;
use crate::strat::utils::AsservHelper;

/// Sim-only convenience pose: at start the simulator teleports the
/// robot here so the Back face roughly faces the calibration wall.
/// IRL the operator wedges the chassis "à peu près" face to the wall;
/// the algorithm doesn't read these constants — it discovers the true
/// pose from the lidars.
pub const TELEPORT_X_MM: f32 = 500.0;
pub const TELEPORT_Y_MM: f32 = 0.0;

/// Settle time after every `goto_a` so the cached lidar reading is
/// guaranteed to come from a fresh CAN frame *after* the rotation
/// finishes. The sim publishes ground-lidar at 250 ms; > 2× margin.
const SETTLE_MS: u64 = 800;

/// Phase 1 — symmetry search around an initial heading. Sweep
/// ±SYM_RANGE_DEG by SYM_STEP_DEG, fit (r_a - r_b)(α) by linear
/// regression, return the α where the difference crosses zero.
///
/// The window has to be wide enough to swallow two combined effects:
///   - mounting tilt of the face vs nominal (Back/Left/Right not at
///     exactly 0/+120/-120° because each face has its own mech drift),
///   - intra-face lidar asymmetry (the two lanes are not perfect
///     mirrors of each other across the face axis).
/// On the sim alone, both effects together can land the symmetric α
/// at ~10° off nominal; ±10° gives a safety margin without making
/// the sweep run forever (21 samples × 800 ms settle ≈ 17 s).
const SYM_RANGE_DEG: f32 = 10.0;
const SYM_STEP_DEG: f32 = 1.0;
/// Maximum drift from the caller's `alpha_center` we accept across
/// retry attempts. If the extrapolated α* keeps walking past this
/// envelope, something is wrong with the setup (operator placed the
/// robot too far off, beam not hitting the wall, etc.) and we abort.
const SYM_MAX_DRIFT_DEG: f32 = 20.0;

/// Phase 2 — LSQ sweep around α*. Wider range than phase 1 so the
/// asymmetry between α and -α terms is large enough to separate D
/// from x_i in the normal equations, AND so the grid search on
/// θ_face has enough leverage to disambiguate it from the (x, y) of
/// each lane (otherwise θ_face / position correlate and the LSQ
/// finds a local min a few degrees off truth).
const FIT_RANGE_DEG: f32 = 25.0;
const FIT_STEP_DEG: f32 = 3.0;

/// Validity filter on raw distances. No CAL_X dependency: tight enough
/// to drop the LD06 "no return" sentinel (= max_range_mm), loose enough
/// to accept whatever distance the robot ends up at vs the wall.
const MIN_VALID_MM: u16 = 50;
const MAX_VALID_MM: u16 = 900;
/// Signal-quality floor — 0 accepts everything; raise if IRL noise
/// shows up.
const SQ_MIN: u16 = 0;

/// Half-spread of the two lidars within a face: ±30° (60° between the
/// lanes). Mechanically fixed by the carrier board geometry, identical
/// across the three faces.
const DELTA_HALF: f32 = core::f32::consts::FRAC_PI_6;

/// Auto-calibration of the 6 ground lidars.
///
/// The routine doesn't assume the robot is at a known pose — only
/// that the **Back face is roughly parallel to a flat wall** at the
/// start (±5° is fine). It discovers everything else.
///
/// Procedure
/// =========
///   1. Phase 1 Back: sweep ±5° around the initial heading, find the
///      α where r₁ == r₄. That α defines the robot's angular zero;
///      we `reset_position` so future α reads are referenced to it.
///   2. Phase 2 Back: wider sweep (±15°), LSQ in 3 unknowns
///      (D, x_back, y_back) using the symmetry x₁ = x₄ = x_back,
///      y₁ = -y₄ = y_back (forced by Phase 1's α=0 convention).
///   3. Rotate ~+120° toward the Left face, repeat Phase 1 + Phase 2.
///      Side faces have 5 unknowns (D, x_a, y_a, x_b, y_b) — no
///      symmetry assumption (face may be tilted).
///   4. Rotate ~-120° from Back toward the Right face, same.
///
/// Per-face equation, for lidar i at body pose (x_i, y_i, θ_i),
/// robot heading α, wall at world `X = X_wall < X_r`, and
/// `D = X_r - X_wall > 0`:
///
/// ```text
/// r_i · cos(α + θ_i) = -D - cos(α)·x_i + sin(α)·y_i
/// ```
///
/// The Phase 1 α* of each face fixes that face's mean angle in the
/// robot frame: `θ_face = π - α*` (the bisector of the two beams
/// points toward the wall when r_a == r_b). Then θ_lane0 =
/// θ_face + π/6 and θ_lane1 = θ_face - π/6 (mechanically fixed
/// 60° spread).
pub fn ground_lidars<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
) {
    log::info!(
        "[ground-cal] start — sim teleport to ({TELEPORT_X_MM:.0}, {TELEPORT_Y_MM:.0}, 0) \
         (algo ignores these values, IRL operator pre-positions manually)"
    );
    asserv.teleport(TELEPORT_X_MM, TELEPORT_Y_MM, 0.0);

    // -------- Back: phase 1 (find α_back_zero in mech/world frame) --------
    let alpha_back_zero = match find_symmetry_angle(asserv, sensors, RobotSide::Back, 0.0) {
        Some(a) => a,
        None => {
            log::error!("[ground-cal] Back phase 1 failed (no symmetric heading found)");
            return;
        }
    };
    log::info!(
        "[ground-cal] Back α_back_zero = {:.4} rad ({:.3}°) — back-symmetric pose in mech gyro",
        alpha_back_zero, alpha_back_zero.to_degrees()
    );
    // Drive to the symmetric pose then re-zero the gyro. From now on
    // pose.a == 0 ↔ Back symmetric. Lidar coords output below are in
    // this "BA" (back-aligned) frame: rotated by α_back_zero from the
    // mech CAD frame. θ_BA = θ_mech + α_back_zero, (x,y)_BA = R(α_back_zero)·(x,y)_mech.
    asserv.goto_a(alpha_back_zero).ok();
    std::thread::sleep(Duration::from_millis(SETTLE_MS));
    let pos = asserv.position();
    asserv.reset_position(pos.x, pos.y, 0.0);

    // -------- Back: phase 2 (5-unknown LSQ + free θ_face) --------
    let back = match calibrate_face(asserv, sensors, RobotSide::Back, 0.0) {
        Some(v) => v,
        None => {
            log::error!("[ground-cal] Back phase 2 failed");
            return;
        }
    };

    // -------- Left: rotate to BA α ≈ +2π/3 --------
    let left_target = core::f32::consts::FRAC_PI_3 * 2.0;
    let left = match calibrate_face(asserv, sensors, RobotSide::Left, left_target) {
        Some(v) => v,
        None => {
            log::error!("[ground-cal] Left calibration failed");
            return;
        }
    };

    // -------- Return to BA α = 0 between sides --------
    // Re-anchor on the back-symmetric pose before rotating to Right
    // so cumulative angular drift (gyro bias IRL, PID rotation
    // residuals) doesn't compound across the two side rotations. We
    // also re-run a quick Phase 1 Back to detect and correct gyro
    // drift accrued during the Left calibration.
    log::info!("[ground-cal] returning to BA α=0 to re-anchor before Right");
    asserv.goto_a(0.0).ok();
    std::thread::sleep(Duration::from_millis(SETTLE_MS));
    let alpha_drift = match find_symmetry_angle(asserv, sensors, RobotSide::Back, 0.0) {
        Some(a) => a,
        None => {
            log::error!("[ground-cal] re-anchor Phase 1 Back failed");
            return;
        }
    };
    log::info!(
        "[ground-cal] re-anchor: Back α* drifted by {:.3}° since the original reset \
         — re-zeroing the gyro",
        alpha_drift.to_degrees(),
    );
    asserv.goto_a(alpha_drift).ok();
    std::thread::sleep(Duration::from_millis(SETTLE_MS));
    let pos = asserv.position();
    asserv.reset_position(pos.x, pos.y, 0.0);

    // -------- Right: BA α ≈ -2π/3 --------
    let right_target = -core::f32::consts::FRAC_PI_3 * 2.0;
    let right = match calibrate_face(asserv, sensors, RobotSide::Right, right_target) {
        Some(v) => v,
        None => {
            log::error!("[ground-cal] Right calibration failed");
            return;
        }
    };

    // -------- Validation summary --------
    log::info!(
        "[ground-cal] D summary: back={:.1} left={:.1} right={:.1} mm \
         (should be close — same wall, same robot center)",
        back.d, left.d, right.d,
    );
    log::info!(
        "[ground-cal] face-orientation residuals (BA frame, post Back-reset): \
         back α* = {:.3}°, left α* − 2π/3 = {:.3}°, right α* + 2π/3 = {:.3}°",
        wrap_pi(back.alpha_star).to_degrees(),
        wrap_pi(left.alpha_star - left_target).to_degrees(),
        wrap_pi(right.alpha_star - right_target).to_degrees(),
    );

    // -------- Render GroundLidarConf for paste into main.rs --------
    // Module 0 = Left, Module 1 = Back, Module 2 = Right (per
    // RobotSideModule::module mapping).
    println!("// Paste into galipeur/src/main.rs (GroundLidarConf):");
    println!("GroundLidarConf {{");
    println!("    modules: [");
    println!("        // Module 0 (Left): lane 0 / lane 1");
    println!("        [");
    print_pose("            ", left.pose_a);
    print_pose("            ", left.pose_b);
    println!("        ],");
    println!("        // Module 1 (Back): lane 0 / lane 1");
    println!("        [");
    print_pose("            ", back.pose_a);
    print_pose("            ", back.pose_b);
    println!("        ],");
    println!("        // Module 2 (Right): lane 0 / lane 1");
    println!("        [");
    print_pose("            ", right.pose_a);
    print_pose("            ", right.pose_b);
    println!("        ],");
    println!("    ],");
    println!("}}");
}

/// Result of a per-face calibration.
struct FaceFit {
    d: f32,
    alpha_star: f32,
    pose_a: (f32, f32, f32), // (x, y, θ) of lane 0 in BA frame
    pose_b: (f32, f32, f32), // (x, y, θ) of lane 1 in BA frame
}

/// Phase 1 + Phase 2 for any face in the BA frame.
///
/// Arguments:
/// - `alpha_target_ba`: BA-frame heading to drive to before Phase 1.
///   Back: 0 (already there post-reset). Left/Right: ±2π/3.
///
/// θ_face is fitted as a free parameter (only Δ = 60° between the two
/// lanes is enforced). We seed the grid search with the formula
/// `π - α*_BA` and refine around it; the search absorbs both face
/// mounting tilt and the small residual due to in-face (x, y)
/// asymmetry.
fn calibrate_face<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
    side: RobotSide,
    alpha_target_ba: f32,
) -> Option<FaceFit> {
    if alpha_target_ba.abs() > 1e-3 {
        // Back is already at BA-α=0 from the reset; only Sides need a rotation.
        asserv.goto_a(alpha_target_ba).ok()?;
        std::thread::sleep(Duration::from_millis(SETTLE_MS));
    }

    let alpha_star = find_symmetry_angle(asserv, sensors, side, alpha_target_ba)?;
    log::info!(
        "[ground-cal] {side:?} α* (BA) = {:.4} rad ({:.3}°), Δ vs target = {:.3}°",
        alpha_star,
        alpha_star.to_degrees(),
        wrap_pi(alpha_star - alpha_target_ba).to_degrees(),
    );

    let samples = collect_phase2_samples(asserv, sensors, side, alpha_star);
    log::info!("[ground-cal] {side:?} phase 2: {} samples", samples.len());

    // Seed the θ_face grid search with the balance-condition formula.
    let theta_face_seed = core::f32::consts::PI - alpha_star;
    let (theta_face, d, [x_a, y_a, x_b, y_b], rms) =
        fit_face_free_theta(&samples, theta_face_seed)?;
    log::info!(
        "[ground-cal] {side:?} fit: D={:.1} mm, lane0=({:.2},{:.2}) lane1=({:.2},{:.2}) \
         θ_face={:.3}° (seed {:.3}°, refined Δ={:.3}°), residual RMS={:.2} mm",
        d, x_a, y_a, x_b, y_b,
        theta_face.to_degrees(),
        theta_face_seed.to_degrees(),
        (theta_face - theta_face_seed).to_degrees(),
        rms,
    );

    Some(FaceFit {
        d,
        alpha_star,
        pose_a: (x_a, y_a, theta_face + DELTA_HALF),
        pose_b: (x_b, y_b, theta_face - DELTA_HALF),
    })
}

/// Read the two lanes of one module as f32 mm, after the caller has
/// already let the lidar settle. Returns None if either lane is out
/// of range or has insufficient signal quality.
fn read_pair<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    side: RobotSide,
) -> Option<(f32, f32)> {
    let m = sensors.ground_lidar_all()[side.module() as usize];
    let v = |d: u16, sq: u16| -> Option<f32> {
        if d >= MIN_VALID_MM && d < MAX_VALID_MM && sq >= SQ_MIN {
            Some(d as f32)
        } else {
            None
        }
    };
    Some((v(m.distance_0, m.sq_0)?, v(m.distance_1, m.sq_1)?))
}

/// Phase 1: sweep around `alpha_center`, fit `(r_a - r_b)(α)` by
/// linear regression, return the α where the diff crosses zero.
///
/// Linear (rather than parabolic on |diff|) because near the
/// symmetric pose the diff itself is approximately linear in α and
/// crosses zero with a non-zero slope — fitting the diff directly is
/// better conditioned than fitting its absolute value.
///
/// If the extrapolated α* falls outside the swept window, we retry
/// the sweep recentered on it (up to 2 retries). This handles cases
/// where the combined Back-reset offset + face tilt push the
/// symmetric pose well past the nominal Δ between faces.
fn find_symmetry_angle<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
    side: RobotSide,
    alpha_center: f32,
) -> Option<f32> {
    let max_drift = SYM_MAX_DRIFT_DEG.to_radians();
    let mut center = alpha_center;
    for attempt in 0..3 {
        match sweep_and_fit(asserv, sensors, side, center)? {
            FitOutcome::InWindow(a) => return Some(a),
            FitOutcome::Outside(extrapolated) => {
                if wrap_pi(extrapolated - alpha_center).abs() > max_drift {
                    log::warn!(
                        "[ground-cal] phase1 {side:?}: extrapolated α* = {:.3}° \
                         drifted > ±{:.1}° from initial center {:.3}° — aborting",
                        extrapolated.to_degrees(),
                        SYM_MAX_DRIFT_DEG,
                        alpha_center.to_degrees(),
                    );
                    return None;
                }
                log::info!(
                    "[ground-cal] phase1 {side:?} attempt {} window missed — \
                     recentering on {:.3}°",
                    attempt + 1,
                    extrapolated.to_degrees(),
                );
                center = extrapolated;
            }
        }
    }
    log::warn!("[ground-cal] phase1 {side:?}: did not converge after 3 attempts");
    None
}

enum FitOutcome {
    InWindow(f32),
    Outside(f32),
}

fn sweep_and_fit<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
    side: RobotSide,
    alpha_center: f32,
) -> Option<FitOutcome> {
    let half = SYM_RANGE_DEG.to_radians();
    let step = SYM_STEP_DEG.to_radians();
    let n = (2.0 * half / step).round() as i32 + 1;

    let mut samples: Vec<(f32, f32)> = Vec::with_capacity(n as usize);
    for i in 0..n {
        let alpha = alpha_center - half + (i as f32) * step;
        if asserv.goto_a(alpha).is_err() {
            log::error!("[ground-cal] phase1 goto_a({alpha:.3}) failed");
            return None;
        }
        std::thread::sleep(Duration::from_millis(SETTLE_MS));
        let actual = asserv.position().a;
        if let Some((r0, r1)) = read_pair(sensors, side) {
            log::info!(
                "[ground-cal] phase1 {side:?} α={:.4} ({:.2}°) r0={:.0} r1={:.0} diff={:.1}",
                actual, actual.to_degrees(), r0, r1, r0 - r1,
            );
            samples.push((actual, r0 - r1));
        } else {
            log::warn!("[ground-cal] phase1 {side:?} α={:.4} dropped (out of range)", actual);
        }
    }

    if samples.len() < 3 {
        log::warn!("[ground-cal] phase1 {side:?}: only {} valid samples", samples.len());
        return None;
    }
    // Linear fit diff = a·α + b  →  zero at α = -b/a.
    let nf = samples.len() as f32;
    let sx: f32 = samples.iter().map(|s| s.0).sum();
    let sy: f32 = samples.iter().map(|s| s.1).sum();
    let sxx: f32 = samples.iter().map(|s| s.0 * s.0).sum();
    let sxy: f32 = samples.iter().map(|s| s.0 * s.1).sum();
    let denom = nf * sxx - sx * sx;
    if denom.abs() < 1e-9 {
        return None;
    }
    let a = (nf * sxy - sx * sy) / denom;
    let b = (sy - a * sx) / nf;
    if a.abs() < 1e-3 {
        // Slope nearly zero → can't locate the crossing precisely.
        log::warn!("[ground-cal] phase1 {side:?}: slope too flat ({a:.4})");
        return None;
    }
    let star = -b / a;
    if wrap_pi(star - alpha_center).abs() <= half + step {
        Some(FitOutcome::InWindow(star))
    } else {
        Some(FitOutcome::Outside(star))
    }
}

/// Wrap an angular delta to (-π, π]. Used to compute angular
/// distances that survive 2π winding-number differences in raw gyro
/// readings (e.g., target = -2π/3 ≈ -2.09 rad vs actual = +4.19 rad
/// representing the same physical heading).
fn wrap_pi(delta: f32) -> f32 {
    let tau = core::f32::consts::TAU;
    let pi = core::f32::consts::PI;
    let d = delta.rem_euclid(tau);
    if d > pi { d - tau } else { d }
}

/// Phase 2: wider sweep around `alpha_star`, return all valid
/// `(actual_α, r_a, r_b)` triples.
fn collect_phase2_samples<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
    side: RobotSide,
    alpha_star: f32,
) -> Vec<(f32, f32, f32)> {
    let half = FIT_RANGE_DEG.to_radians();
    let step = FIT_STEP_DEG.to_radians();
    let n = (2.0 * half / step).round() as i32 + 1;

    let mut samples = Vec::with_capacity(n as usize);
    for i in 0..n {
        let alpha = alpha_star - half + (i as f32) * step;
        if asserv.goto_a(alpha).is_err() {
            log::error!("[ground-cal] phase2 goto_a({alpha:.3}) failed");
            continue;
        }
        std::thread::sleep(Duration::from_millis(SETTLE_MS));
        let actual = asserv.position().a;
        if let Some((r_a, r_b)) = read_pair(sensors, side) {
            log::info!(
                "[ground-cal] phase2 {side:?} α={:.4} ({:.2}°) r_a={:.0} r_b={:.0}",
                actual, actual.to_degrees(), r_a, r_b,
            );
            samples.push((actual, r_a, r_b));
        }
    }
    samples
}

/// Inner LSQ given a fixed θ_face: 5 unknowns (D, x_a, y_a, x_b, y_b).
/// Returns the parameters and the squared residual sum.
///
/// Per sample (α, r_a, r_b), with θ_a = θ_face + π/6, θ_b = θ_face - π/6:
///   row a:  [-1, -cos(α), +sin(α), 0, 0] · u = r_a · cos(α + θ_a)
///   row b:  [-1, 0, 0, -cos(α), +sin(α)] · u = r_b · cos(α + θ_b)
fn fit_face_inner(
    samples: &[(f32, f32, f32)],
    theta_face: f32,
) -> Option<(f32, [f32; 4], f64)> {
    if samples.len() < 3 {
        return None;
    }
    let theta_a = (theta_face + DELTA_HALF) as f64;
    let theta_b = (theta_face - DELTA_HALF) as f64;

    let mut ata = [[0.0_f64; 5]; 5];
    let mut atv = [0.0_f64; 5];

    for &(alpha, r_a, r_b) in samples {
        let alpha = alpha as f64;
        let ca = alpha.cos();
        let sa = alpha.sin();
        let row_a = [-1.0, -ca, sa, 0.0, 0.0];
        let val_a = (r_a as f64) * (alpha + theta_a).cos();
        let row_b = [-1.0, 0.0, 0.0, -ca, sa];
        let val_b = (r_b as f64) * (alpha + theta_b).cos();
        for i in 0..5 {
            for j in 0..5 {
                ata[i][j] += row_a[i] * row_a[j] + row_b[i] * row_b[j];
            }
            atv[i] += row_a[i] * val_a + row_b[i] * val_b;
        }
    }

    let u = solve_linear::<5>(ata, atv)?;

    // Compute squared residual to feed the outer θ_face search.
    let mut rss = 0.0_f64;
    for &(alpha, r_a, r_b) in samples {
        let alpha = alpha as f64;
        let ca = alpha.cos();
        let sa = alpha.sin();
        let res_a = -u[0] - ca * u[1] + sa * u[2] - (r_a as f64) * (alpha + theta_a).cos();
        let res_b = -u[0] - ca * u[3] + sa * u[4] - (r_b as f64) * (alpha + theta_b).cos();
        rss += res_a * res_a + res_b * res_b;
    }

    Some((
        u[0] as f32,
        [u[1] as f32, u[2] as f32, u[3] as f32, u[4] as f32],
        rss,
    ))
}

/// LSQ for any face with θ_face as a FREE parameter, only the angular
/// spread Δ = 60° between the two lanes is enforced.
///
/// Strategy: grid-search θ_face around an initial estimate, running
/// the inner 5-unknown linear LSQ at each candidate, picking the
/// θ_face that minimizes residual. Coarse-then-fine: first pass at
/// 0.5° step over ±10° (41 trials), then 0.05° step over ±0.5° (21
/// trials). Total ≈ 62 small linear solves — cheap.
///
/// Returns (θ_face_best, D, [x_a, y_a, x_b, y_b], residual_rms_mm²).
fn fit_face_free_theta(
    samples: &[(f32, f32, f32)],
    theta_face_init: f32,
) -> Option<(f32, f32, [f32; 4], f32)> {
    let mut best: Option<(f32, f32, [f32; 4], f64)> = None;
    let try_candidate = |theta: f32, best: &mut Option<(f32, f32, [f32; 4], f64)>| {
        if let Some((d, fits, rss)) = fit_face_inner(samples, theta) {
            if best.map_or(true, |b| rss < b.3) {
                *best = Some((theta, d, fits, rss));
            }
        }
    };

    // Coarse pass.
    let coarse_half = 10.0_f32.to_radians();
    let coarse_step = 0.5_f32.to_radians();
    let coarse_n = ((2.0 * coarse_half / coarse_step).round() as i32) + 1;
    for i in 0..coarse_n {
        let theta = theta_face_init - coarse_half + (i as f32) * coarse_step;
        try_candidate(theta, &mut best);
    }
    let coarse_best_theta = best.as_ref().map(|b| b.0)?;

    // Fine pass around the coarse minimum.
    let fine_half = 0.5_f32.to_radians();
    let fine_step = 0.05_f32.to_radians();
    let fine_n = ((2.0 * fine_half / fine_step).round() as i32) + 1;
    for i in 0..fine_n {
        let theta = coarse_best_theta - fine_half + (i as f32) * fine_step;
        try_candidate(theta, &mut best);
    }

    best.map(|(t, d, f, rss)| {
        let n_eq = (2 * samples.len()) as f64;
        let rms = (rss / n_eq).sqrt() as f32;
        (t, d, f, rms)
    })
}

/// Solve `A · x = b` for square A (size N) by Gaussian elimination with
/// partial pivoting. Returns None if the matrix is singular.
fn solve_linear<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for k in 0..N {
        // Pivot: row with largest |a[i][k]| for i ≥ k.
        let mut piv = k;
        let mut piv_v = a[k][k].abs();
        for i in (k + 1)..N {
            let v = a[i][k].abs();
            if v > piv_v {
                piv = i;
                piv_v = v;
            }
        }
        if piv_v < 1e-12 {
            return None;
        }
        if piv != k {
            a.swap(k, piv);
            b.swap(k, piv);
        }
        // Eliminate.
        for i in (k + 1)..N {
            let f = a[i][k] / a[k][k];
            for j in k..N {
                a[i][j] -= f * a[k][j];
            }
            b[i] -= f * b[k];
        }
    }
    // Back-substitute.
    let mut x = [0.0_f64; N];
    for i in (0..N).rev() {
        let mut s = b[i];
        for j in (i + 1)..N {
            s -= a[i][j] * x[j];
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

fn print_pose(indent: &str, pose: (f32, f32, f32)) {
    let (x, y, t) = pose;
    println!(
        "{indent}GroundLidarPose {{ x: {x:.2}, y: {y:.2}, \
         theta: {:.3}_f32.to_radians() }},",
        t.to_degrees(),
    );
}
