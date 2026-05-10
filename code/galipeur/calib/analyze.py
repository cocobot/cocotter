#!/usr/bin/env python3
"""Fit ground-lidar poses from a galipeur sweep dump.

Pipe the galipeur stdout into this script (or pass a log file as
argument). It picks up the CSV between the `# ground-cal samples —
BEGIN` / `END` markers, drops invalid hits, runs an LSQ fit per lidar
and prints a ready-to-paste `GroundLidarConf` block.

Model (wall at asserv.x = 0, robot at x = CAL_X):
    reading * cos(theta_R + theta_L) =
        -(CAL_X + cos(theta_R) * x_L - sin(theta_R) * y_L)

Linear in (x_L, y_L) for fixed theta_L. Grid-search theta_L (1° then
0.05° refinement), pick the smallest residual.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import numpy as np

# Usable distance window — drop the lidar's "no hit" sentinel and
# clearly-too-close readings (off-fixture clutter).
MAX_VALID_MM = 900
MIN_VALID_MM = 100.0

# Asymmetric fit window per lane, in degrees relative to the expected
# θ_min for the target wall. The two lidars on a single physical
# bracket see the wall through different mounting offsets, so the
# angular range is shifted per lane — but identical across modules
# (they are mechanically identical, just rotated by 120°).
LANE_FIT_WINDOW_DEG: list[tuple[float, float]] = [
    (-40.0, +60.0),  # lane 0 — lidar idx 0, 2, 4
    (-60.0, +40.0),  # lane 1 — lidar idx 1, 3, 5
]

# Expected θ_min (robot heading at which each lidar sees the
# calibration wall at minimum distance), derived from module angular
# position on the chassis.  Modules are at 0° (back), 120°, 240°.
# CAN modules 0 and 2 each have one lidar per face (bracket pairs):
#   Face 120° (Left):  L0 (module 0 lane 0) + L5 (module 2 lane 1)
#   Face back (0°):    L2 (module 1 lane 0) + L3 (module 1 lane 1)
#   Face 240° (Right): L1 (module 0 lane 1) + L4 (module 2 lane 0)
MODULE_THETA_MIN_DEG: list[float] = [
    120.0,  # L0 — module 0 lane 0 → Left face
    240.0,  # L1 — module 0 lane 1 → Right face
      0.0,  # L2 — module 1 lane 0 → Back face
      0.0,  # L3 — module 1 lane 1 → Back face
    240.0,  # L4 — module 2 lane 0 → Right face
    120.0,  # L5 — module 2 lane 1 → Left face
]


@dataclass
class Sample:
    pass_name: str  # "fwd" or "rev"
    theta: float    # rad
    readings: list[float | None]  # length 6, None if invalid


@dataclass
class TopScanPoint:
    """One LD06 measurement at a given sweep step."""
    pass_name: str
    theta: float       # asserv heading (rad)
    step_idx: int
    angle_deg: float   # LD06 angle in body frame (deg)
    dist_mm: float
    intensity: int


def parse(stream) -> tuple[list[Sample], list[TopScanPoint]]:
    """Parse both ground-cal (14-column) and top-cal (6-column) lines.

    Returns (ground_samples, top_scan_points).
    """
    in_block = False
    saw_header = False
    samples: list[Sample] = []
    top_points: list[TopScanPoint] = []
    for raw in stream:
        line = raw.strip()
        if not line:
            continue
        if line.startswith("# ground-cal samples — BEGIN"):
            in_block = True
            continue
        if line.startswith("# ground-cal samples — END"):
            in_block = False
            continue
        if not in_block or line.startswith("#"):
            continue
        if not saw_header:
            saw_header = True
            continue
        parts = line.split(",")
        if len(parts) == 14:
            try:
                pass_name = parts[0]
                theta = float(parts[1])
                readings: list[float | None] = []
                for k in range(6):
                    d = int(parts[2 + 2 * k])
                    if MIN_VALID_MM <= d <= MAX_VALID_MM:
                        readings.append(float(d))
                    else:
                        readings.append(None)
                samples.append(Sample(pass_name, theta, readings))
            except ValueError:
                continue
        elif len(parts) > 3 and ':' in parts[3]:
            # Packed LD06 scan: pass,theta,step_idx,a1:d1,a2:d2,...
            try:
                pass_name = parts[0]
                theta = float(parts[1])
                step_idx = int(parts[2])
                for token in parts[3:]:
                    a_s, d_s = token.split(':')
                    top_points.append(TopScanPoint(
                        pass_name=pass_name,
                        theta=theta,
                        step_idx=step_idx,
                        angle_deg=float(a_s),
                        dist_mm=float(d_s),
                        intensity=0,
                    ))
            except ValueError:
                continue
        elif len(parts) == 6:
            # Legacy per-point LD06 format (6 columns).
            try:
                top_points.append(TopScanPoint(
                    pass_name=parts[0],
                    theta=float(parts[1]),
                    step_idx=int(parts[2]),
                    angle_deg=float(parts[3]),
                    dist_mm=float(parts[4]),
                    intensity=int(parts[5]),
                ))
            except ValueError:
                continue
    return samples, top_points


def wrap_pi(x: float) -> float:
    return (x + math.pi) % (2 * math.pi) - math.pi


def solve_lsq(points: list[tuple[float, float]], theta_l: float,
              cal_x: float | None = None,
              ) -> tuple[float, float, float, float] | None:
    """LSQ solve for (x_L, y_L[, CAL_X]) at fixed theta_L.

    Model: -cos(θ)*x_L + sin(θ)*y_L - CAL_X = r*cos(θ + θ_L)

    If *cal_x* is given it is fixed; otherwise it is estimated as a
    third unknown.  Returns (x_L, y_L, cal_x, rss).
    """
    n = len(points)
    if n < (3 if cal_x is not None else 4):
        return None
    ncols = 2 if cal_x is not None else 3
    A = np.empty((n, ncols))
    b = np.empty(n)
    for k, (theta, r) in enumerate(points):
        A[k, 0] = -math.cos(theta)
        A[k, 1] = math.sin(theta)
        rhs = r * math.cos(theta + theta_l)
        if cal_x is not None:
            b[k] = rhs + cal_x          # move known CAL_X to RHS
        else:
            A[k, 2] = -1.0
            b[k] = rhs
    res = np.linalg.lstsq(A, b, rcond=None)
    rss = float(np.sum((A @ res[0] - b) ** 2))
    if cal_x is not None:
        x_l, y_l = res[0]
        return float(x_l), float(y_l), cal_x, rss
    else:
        x_l, y_l, cx = res[0]
        return float(x_l), float(y_l), float(cx), rss


def _longest_contiguous_run(mask: list[bool]) -> tuple[int, int]:
    """Return (start, end) of the longest contiguous True run in *mask*.

    Returns (0, 0) if no True values exist.
    """
    best_start, best_len = 0, 0
    i = 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            if j - i > best_len:
                best_start, best_len = i, j - i
            i = j
        else:
            i += 1
    return best_start, best_start + best_len


def _in_fit_window(theta: float, lidar_idx: int, lane: int) -> bool:
    """Check whether *theta* falls inside the fit window for *lidar_idx*."""
    theta_min_expected = math.radians(MODULE_THETA_MIN_DEG[lidar_idx])
    lo_deg, hi_deg = LANE_FIT_WINDOW_DEG[lane]
    delta = wrap_pi(theta - theta_min_expected)
    return math.radians(lo_deg) <= delta <= math.radians(hi_deg)


def select_fit_window(points: list[tuple[float, float]], lane: int,
                      lidar_idx: int = 0
                      ) -> list[tuple[float, float]]:
    """Apply the per-lane asymmetric window around the *expected* θ_min.

    Uses `MODULE_THETA_MIN_DEG[lidar_idx]` so that, in a multi-wall
    setup, each lidar only fits data from the target wall.  The window
    shape is per-lane (same across modules — they are mechanically
    identical, just rotated by 120°).

    To avoid mixing data from discontinuous sweep segments (e.g. the
    start and end of a 1.5-turn sweep both fall inside the window for
    the back module at θ_min=0°), only the longest contiguous run of
    in-window samples is kept.
    """
    if not points:
        return []

    mask = [_in_fit_window(t, lidar_idx, lane) for (t, _) in points]
    s, e = _longest_contiguous_run(mask)
    return [points[k] for k in range(s, e)]


def fit_lidar(points: list[tuple[float, float]], lane: int = 0,
              lidar_idx: int = 0, cal_x: float | None = None,
              ) -> tuple[float, float, float, float, float, float] | None:
    """Returns (x_mm, y_mm, theta_rad, cal_x_mm, r_min_mm, theta_min_rad).

    `lane` selects which `LANE_FIT_WINDOW_DEG` entry to use — pass
    `lidar_idx % 2` from the caller.  If *cal_x* is ``None`` it is
    estimated jointly; otherwise it is held fixed (global-fit mode).
    """
    if len(points) < 5:
        return None

    valid = select_fit_window(points, lane, lidar_idx)
    if len(valid) < 5:
        return None

    # Coarse grid search over theta_L.
    best: tuple[float, float, float, float, float] | None = None  # rss, x, y, cal_x, theta_l
    for i in range(360):
        theta_l = math.radians(i)
        sol = solve_lsq(valid, theta_l, cal_x=cal_x)
        if sol is None:
            continue
        x_l, y_l, cx, rss = sol
        if best is None or rss < best[0]:
            best = (rss, x_l, y_l, cx, theta_l)
    if best is None:
        return None
    # Fine refine ±1° around the coarse minimum.
    _, _, _, _, t0 = best
    d = -1.0
    while d <= 1.0:
        theta_l = t0 + math.radians(d)
        sol = solve_lsq(valid, theta_l, cal_x=cal_x)
        if sol is not None:
            x_l, y_l, cx, rss = sol
            if rss < best[0]:
                best = (rss, x_l, y_l, cx, theta_l)
        d += 0.05
    _, x_l, y_l, cx, theta_l = best

    # Break the sign degeneracy: (x, y, θ_L, CAL_X) and
    # (-x, -y, θ_L+π, -CAL_X) give identical predictions.
    # The robot is on the +x side of the wall, so CAL_X must be > 0.
    if cx < 0:
        x_l = -x_l
        y_l = -y_l
        theta_l += math.pi
        cx = -cx

    # r_min / theta_min from the fitted model — sub-mm precision.
    r_min = math.inf
    theta_min = 0.0
    for i in range(3600):
        th = math.radians(i * 0.1)
        denom = math.cos(th + theta_l)
        if abs(denom) < 1e-3:
            continue
        r = -(cx + math.cos(th) * x_l - math.sin(th) * y_l) / denom
        if 0.0 < r < r_min:
            r_min = r
            theta_min = th

    theta_l_wrapped = wrap_pi(theta_l)
    return x_l, y_l, theta_l_wrapped, cx, r_min, theta_min


def extract_wall_distance_from_scan(
    top_points: list[TopScanPoint],
) -> list[tuple[str, float, int, float]]:
    """Extract per-step wall distance from LD06 scans.

    For each step, find the wall (closest cluster near the expected
    direction) and return the perpendicular distance.

    Returns list of (pass_name, theta_rad, step_idx, wall_dist_mm).
    """
    from collections import defaultdict
    by_step: dict[int, list[TopScanPoint]] = defaultdict(list)
    for p in top_points:
        by_step[p.step_idx].append(p)

    results: list[tuple[str, float, int, float]] = []
    for step_idx in sorted(by_step):
        pts = by_step[step_idx]
        if not pts:
            continue
        pass_name = pts[0].pass_name
        theta = pts[0].theta  # asserv heading

        # Wall at world x=0, robot at (CAL_X, 0) heading θ.
        # In body frame, the wall direction is at angle (180° − θ) (deg).
        wall_dir_deg = math.degrees(math.pi - theta) % 360.0

        # Select points within ±30° of the wall direction and within
        # a reasonable distance range.
        candidates: list[tuple[float, float]] = []  # (angle_rad_body, dist)
        for p in pts:
            angle_diff = (p.angle_deg - wall_dir_deg + 180) % 360 - 180
            if abs(angle_diff) < 30 and 50 < p.dist_mm < 2000:
                candidates.append((math.radians(p.angle_deg), p.dist_mm))

        if len(candidates) < 3:
            continue

        # Perpendicular distance to wall = d * cos(angle - wall_dir)
        wall_dir_rad = math.radians(wall_dir_deg)
        perp_dists = [d * math.cos(a - wall_dir_rad)
                      for a, d in candidates]
        # Median is robust to outliers from other objects.
        wall_dist = float(np.median(perp_dists))
        results.append((pass_name, theta, step_idx, wall_dist))
    return results


def fit_top_drift(
    wall_dists: list[tuple[str, float, int, float]],
) -> tuple[float, float, float, float, float] | None:
    """Fit d = d₀ + a·cos(θ) + b·sin(θ) + slope·step_idx.

    The sinusoidal terms capture the LD06 offset from the center of
    rotation; the linear term captures the robot drift.

    Returns (d0, a, b, slope, rss) or None.
    """
    n = len(wall_dists)
    if n < 10:
        return None

    A = np.empty((n, 4))
    y = np.empty(n)
    for k, (_, theta, step_idx, dist) in enumerate(wall_dists):
        A[k, 0] = 1.0                # d₀
        A[k, 1] = math.cos(theta)    # a
        A[k, 2] = math.sin(theta)    # b
        A[k, 3] = float(step_idx)    # slope
        y[k] = dist

    res = np.linalg.lstsq(A, y, rcond=None)
    coeffs = res[0]
    rss = float(np.sum((A @ coeffs - y) ** 2))
    return float(coeffs[0]), float(coeffs[1]), float(coeffs[2]), float(coeffs[3]), rss


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?", help="log file (default: stdin)")
    args = ap.parse_args()

    if args.file:
        with open(args.file) as f:
            samples, top_points = parse(f)
    else:
        samples, top_points = parse(sys.stdin)

    if not samples:
        print("no samples found between BEGIN/END markers", file=sys.stderr)
        return 1

    print(f"# parsed {len(samples)} ground samples "
          f"(fwd={sum(1 for s in samples if s.pass_name == 'fwd')}, "
          f"rev={sum(1 for s in samples if s.pass_name == 'rev')}), "
          f"{len(top_points)} LD06 points",
          file=sys.stderr)

    # Prepare per-lidar point lists, tagged with sample index so we
    # can track temporal order for drift correction.
    # Each entry: (theta, r, global_sample_index, pass_name)
    tagged: list[list[tuple[float, float, int, str]]] = [[] for _ in range(6)]
    for idx, s in enumerate(samples):
        for lidar in range(6):
            if s.readings[lidar] is not None:
                tagged[lidar].append(
                    (s.theta, float(s.readings[lidar]), idx, s.pass_name))

    def pts_only(tag_list):
        return [(t, r) for t, r, _, _ in tag_list]

    # ---- Pass 1: independent fit (CAL_X free) to get a consensus. ----
    cal_xs: list[float] = []
    for lidar in range(6):
        fit = fit_lidar(pts_only(tagged[lidar]),
                        lane=lidar % 2, lidar_idx=lidar)
        if fit is not None:
            cal_xs.append(fit[3])
    cal_x_global = float(np.median(cal_xs)) if cal_xs else 300.0
    print(f"# Pass 1 — CAL_X per-lidar: {', '.join(f'{c:.1f}' for c in cal_xs)}",
          file=sys.stderr)
    print(f"# Pass 1 — CAL_X median: {cal_x_global:.1f} mm", file=sys.stderr)

    # ---- Pass 2: drift correction. ----
    # Try LD06-based drift first (independent measurement), fall back
    # to per-lidar residual-based drift if no LD06 data.
    print("# --- Drift correction ---", file=sys.stderr)

    # Build step_idx → sample_idx mapping.  In the fwd pass each step
    # produces 1 ground sample; in the rev pass each step produces
    # SAMPLES_PER_STEP_SLOW (5) ground samples but only 1 LD06 scan.
    # step_idx is monotonic across fwd then rev.

    wall_dists = extract_wall_distance_from_scan(top_points) if top_points else []
    top_drift_fit = fit_top_drift(wall_dists) if wall_dists else None

    if top_drift_fit is not None:
        d0, a, b, slope, rss = top_drift_fit
        n_top = len(wall_dists)
        step_range = wall_dists[-1][2] - wall_dists[0][2]
        total_drift = slope * step_range
        residuals_top = np.array([
            dist - (d0 + a * math.cos(theta) + b * math.sin(theta)
                    + slope * si)
            for _, theta, si, dist in wall_dists])
        std_top = float(np.std(residuals_top))
        print(f"# LD06 drift: {n_top} scans, slope={slope:.3f} mm/step, "
              f"total={total_drift:+.1f} mm, std={std_top:.1f} mm, "
              f"LD06 offset=({a:.1f}, {b:.1f}) mm",
              file=sys.stderr)

        # Apply LD06-based drift correction to ground lidar data.
        # The drift is along the wall-normal (x-axis).  For a ground
        # lidar seeing the wall at angle (θ_R + θ_L), a CAL_X change
        # of Δ changes the reading r by -Δ/cos(θ_R + θ_L).  But we
        # don't know θ_L yet (that's what we're fitting!).  Instead,
        # correct CAL_X directly: subtract the drift from the raw
        # distance projected onto the wall normal.  Since
        #   r·cos(θ_R + θ_L) = -(CAL_X + ...)
        # a CAL_X drift of δ gives:
        #   r_new·cos(θ_R + θ_L) = -(CAL_X + δ + ...)
        # We want to remove δ, so we store the drift per step and
        # let fit_lidar see a corrected CAL_X.  The simplest: correct
        # the raw distances directly.
        #
        # But r depends on θ_L which we don't have yet.  So instead
        # we build a per-step_idx drift-in-CAL_X and pass it to the
        # fitting as a time-varying CAL_X correction.
        #
        # For now, use the approach that works: correct the raw
        # distance readings.  We need θ_L for that, so do a
        # preliminary fit first.
        fits_p2 = []
        for lidar in range(6):
            fit = fit_lidar(pts_only(tagged[lidar]),
                            lane=lidar % 2, lidar_idx=lidar,
                            cal_x=cal_x_global)
            fits_p2.append(fit)

        # Map ground sample index → step_idx.  fwd: 1 sample per
        # step; rev: SAMPLES_PER_STEP_SLOW per step.
        n_fwd = sum(1 for s in samples if s.pass_name == 'fwd')
        sample_to_step: list[int] = []
        # fwd: sample i → step i
        for i in range(n_fwd):
            sample_to_step.append(i)
        # rev: samples come in groups of SAMPLES_PER_STEP_SLOW
        rev_step = n_fwd  # step_idx continues from fwd
        rev_count = 0
        for s in samples:
            if s.pass_name != 'rev':
                continue
            sample_to_step.append(rev_step)
            rev_count += 1
            if rev_count % 5 == 0:  # SAMPLES_PER_STEP_SLOW = 5
                rev_step += 1

        corrected: list[list[tuple[float, float, int, str]]] = [[] for _ in range(6)]
        for lidar in range(6):
            fit = fits_p2[lidar]
            if fit is None:
                corrected[lidar] = list(tagged[lidar])
                continue
            x_l, y_l, theta_l, cx, _, _ = fit

            for theta, r, si, pn in tagged[lidar]:
                step = sample_to_step[si] if si < len(sample_to_step) else 0
                # Drift in wall distance at this step.
                delta_cal_x = slope * step
                # How this affects reading r:
                #   r = -(CAL_X + δ + x_L·cos(θ) - y_L·sin(θ)) / cos(θ + θ_L)
                # We want r without δ:
                #   r_corr = r + δ / cos(θ + θ_L)
                denom = math.cos(theta + theta_l)
                if abs(denom) < 1e-6:
                    corrected[lidar].append((theta, r, si, pn))
                else:
                    corrected[lidar].append((theta, r + delta_cal_x / denom, si, pn))
    else:
        # Fallback: per-lidar residual-based drift correction.
        if top_points:
            print("# LD06 drift fit failed, using residual-based fallback",
                  file=sys.stderr)
        else:
            print("# No LD06 data, using residual-based drift correction",
                  file=sys.stderr)

        fits_p2 = []
        for lidar in range(6):
            fit = fit_lidar(pts_only(tagged[lidar]),
                            lane=lidar % 2, lidar_idx=lidar,
                            cal_x=cal_x_global)
            fits_p2.append(fit)

        corrected = [[] for _ in range(6)]
        for lidar in range(6):
            fit = fits_p2[lidar]
            if fit is None:
                corrected[lidar] = list(tagged[lidar])
                continue
            x_l, y_l, theta_l, cx, _, _ = fit

            for pass_name in ("fwd", "rev"):
                pass_samples = [(t, r, si, pn) for t, r, si, pn in tagged[lidar]
                                if pn == pass_name]

                lane = lidar % 2
                mask = [_in_fit_window(t, lidar, lane)
                        for (t, _, _, _) in pass_samples]
                run_s, run_e = _longest_contiguous_run(mask)

                residuals: list[float] = []
                sample_indices: list[int] = []
                for j in range(run_s, run_e):
                    theta, r, si, _ = pass_samples[j]
                    denom = math.cos(theta + theta_l)
                    if abs(denom) < 1e-6:
                        continue
                    model = -(cx + math.cos(theta) * x_l
                              - math.sin(theta) * y_l) / denom
                    residuals.append(r - model)
                    sample_indices.append(si)

                if len(residuals) < 5:
                    for t, r, si, pn in pass_samples:
                        corrected[lidar].append((t, r, si, pn))
                    continue

                res_arr = np.array(residuals)
                si_arr = np.array(sample_indices, dtype=float)
                slope_r, intercept = np.polyfit(si_arr, res_arr, 1)
                total_drift = slope_r * (si_arr[-1] - si_arr[0])
                std_before = float(np.std(res_arr))
                std_after = float(np.std(res_arr - (slope_r * si_arr + intercept)))
                print(f"#   L{lidar} {pass_name}: {len(residuals):3d} pts, "
                      f"drift={total_drift:+.1f} mm, "
                      f"std {std_before:.2f} → {std_after:.2f} mm",
                      file=sys.stderr)

                for t, r, si, pn in pass_samples:
                    correction = slope_r * si + intercept
                    corrected[lidar].append((t, r - correction, si, pn))

    # ---- Pass 3: final fit with drift-corrected data. ----
    # Re-estimate CAL_X from corrected data, then final fit.
    cal_xs_c: list[float] = []
    for lidar in range(6):
        fit = fit_lidar(pts_only(corrected[lidar]),
                        lane=lidar % 2, lidar_idx=lidar)
        if fit is not None:
            cal_xs_c.append(fit[3])
    cal_x_global = float(np.median(cal_xs_c)) if cal_xs_c else cal_x_global
    print(f"# Pass 3 — CAL_X per-lidar (corrected): "
          f"{', '.join(f'{c:.1f}' for c in cal_xs_c)}", file=sys.stderr)
    print(f"# Pass 3 — CAL_X median: {cal_x_global:.1f} mm", file=sys.stderr)

    fits = []
    for lidar in range(6):
        fit = fit_lidar(pts_only(corrected[lidar]),
                        lane=lidar % 2, lidar_idx=lidar,
                        cal_x=cal_x_global)
        fits.append(fit)
        if fit is None:
            print(f"# lidar {lidar}: FIT FAILED",
                  file=sys.stderr)
        else:
            x, y, t, cx, r_min, th_min = fit
            print(
                f"# lidar {lidar}: x={x:7.2f}  y={y:7.2f}  "
                f"theta={math.degrees(t):7.2f}°  "
                f"r_min={r_min:6.1f} mm @ θ_R={math.degrees(th_min):6.1f}°",
                file=sys.stderr,
            )
    print(f"# CAL_X (global, final): {cal_x_global:.1f} mm", file=sys.stderr)

    # Ready-to-paste block for galipeur/src/main.rs.
    # The calibration body frame is rotated -90° from the asserv body
    # frame, so apply a +90° rotation: x' = -y, y' = x, θ' = θ + 90°.
    #
    # CAN modules 0 and 2 each have one lidar per face, so we regroup
    # by face (swapping lane 1 between modules 0 and 2):
    #   Face Left  (module 0 in config): L0 (CAN m0 lane 0) + L5 (CAN m2 lane 1)
    #   Face Back  (module 1 in config): L2 (CAN m1 lane 0) + L3 (CAN m1 lane 1)
    #   Face Right (module 2 in config): L4 (CAN m2 lane 0) + L1 (CAN m0 lane 1)
    face_lidar_indices = [
        (0, 5),  # Left face
        (2, 3),  # Back face
        (4, 1),  # Right face
    ]
    face_names = ["Left", "Back", "Right"]

    def fmt_pose(fit):
        if fit is None:
            return None
        x, y, t, cal_x, r_min, th_min = fit
        x_a, y_a, t_a = -y, x, t + math.radians(90)
        t_a = wrap_pi(t_a)
        return x_a, y_a, t_a, r_min, th_min

    print("// Paste into galipeur/src/main.rs (GroundLidarConf):")
    print("GroundLidarConf {")
    print("    modules: [")
    for face_idx, (li0, li1) in enumerate(face_lidar_indices):
        print(f"        // {face_names[face_idx]} face: lidar {li0} and lidar {li1}")
        print("        [")
        for idx in (li0, li1):
            pose = fmt_pose(fits[idx])
            if pose is None:
                print(f"            GroundLidarPose {{ x: 0.0, y: 0.0, theta: 0.0 }},  "
                      f"// lidar {idx}: FIT FAILED")
            else:
                x_a, y_a, t_a, r_min, th_min = pose
                print(
                    f"            GroundLidarPose {{ x: {x_a:.2f}, y: {y_a:.2f}, "
                    f"theta: {math.degrees(t_a):.3f}_f32.to_radians() }},  "
                    f"// L{idx} r_min={r_min:.0f} mm @ θ_R={math.degrees(th_min):.1f}°"
                )
        print("        ],")
    print("    ],")
    print("}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
