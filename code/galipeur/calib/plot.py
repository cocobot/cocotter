#!/usr/bin/env python3
"""Render a galipeur ground-lidar sweep log as an interactive HTML.

Usage:
    ./plot.py path/to/sweep.log [-o out.html]

The script parses the CSV between the `# ground-cal samples — BEGIN`
and `END` markers, ignoring all other firmware log spam, and emits a
self-contained Plotly HTML page with:

  * Polar overlay (fwd + rev) per lidar — sanity check of where each
    lidar sees the wall.
  * Distance vs heading (per lidar) — fwd as a line, rev as scatter
    (5 samples per stop) so you can see the repeatability spread.
  * Signal quality (sq) vs heading — useful to drop low-confidence
    points or weight the LSQ.
  * Reverse-pass repeatability: per-stop stdev of the 5 samples,
    aggregated per lidar.
  * Polar 2D scatter of all hit points in the robot frame, assuming a
    point lidar at the origin (no theta_L correction yet — that's
    what the LSQ recovers).
  * Robot top-down view showing each lidar's fitted (x, y) and
    pointing direction — calls into `analyze.fit_lidar`.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Allow `import analyze` when plot.py is run from any CWD.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze  # noqa: E402


MAX_VALID_MM = 900
MIN_VALID_MM = 100.0


def parse(path: Path):
    """Returns dict with arrays per pass:
        passes[name] = {
            theta: (N,) rad,
            d:    (N, 6) raw mm (0 if no hit),
            sq:   (N, 6) sq,
        }
    Lidar columns are 0..5 in CAN order: module 0 lidar 0, module 0
    lidar 1, module 1 lidar 0, ..., module 2 lidar 1.
    """
    in_block = False
    saw_header = False
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("# ground-cal samples — BEGIN"):
            in_block = True
            continue
        if s.startswith("# ground-cal samples — END"):
            in_block = False
            continue
        if not in_block or s.startswith("#"):
            continue
        if not saw_header:
            saw_header = True
            continue
        parts = s.split(",")
        if len(parts) != 14:
            continue
        try:
            pass_name = parts[0]
            theta = float(parts[1])
            d = [int(parts[2 + 2 * k]) for k in range(6)]
            sq = [int(parts[3 + 2 * k]) for k in range(6)]
        except ValueError:
            continue
        rows.append((pass_name, theta, d, sq))

    by_pass: dict[str, dict[str, np.ndarray]] = {}
    for name in ("fwd", "rev"):
        sub = [r for r in rows if r[0] == name]
        if not sub:
            continue
        theta = np.array([r[1] for r in sub], dtype=float)
        d = np.array([r[2] for r in sub], dtype=float)
        sq = np.array([r[3] for r in sub], dtype=float)
        by_pass[name] = {"theta": theta, "d": d, "sq": sq}
    return by_pass


def valid_mask(d: np.ndarray) -> np.ndarray:
    """True where the reading is plausibly the calibration wall."""
    return (d >= MIN_VALID_MM) & (d <= MAX_VALID_MM)


# Opacity used for samples that fail either the validity window or the
# per-lidar `±FIT_WINDOW_DEG` window — i.e. samples that don't end up
# in the LSQ. Keeps them visible for context without drowning the
# participating ones.
DIM_ALPHA = 0.2


def participating_masks(passes):
    """Per-lidar boolean masks marking samples that participate in the
    LSQ fit. Mirrors `analyze.select_fit_window`: valid range +
    asymmetric `LANE_FIT_WINDOW_DEG` window relative to θ_min,
    selected by `lidar_idx % 2` (lane)."""
    out: dict[int, dict[str, np.ndarray]] = {i: {} for i in range(6)}

    for i in range(6):
        thetas: list[np.ndarray] = []
        ds: list[np.ndarray] = []
        splits: list[tuple[str, int]] = []
        for name in ("fwd", "rev"):
            if name in passes:
                thetas.append(passes[name]["theta"])
                ds.append(passes[name]["d"][:, i])
                splits.append((name, len(passes[name]["theta"])))
        if not thetas:
            continue
        theta_all = np.concatenate(thetas)
        d_all = np.concatenate(ds)
        valid_all = valid_mask(d_all)

        offset = 0
        if not valid_all.any():
            for name, n in splits:
                out[i][name] = np.zeros(n, dtype=bool)
                offset += n
            continue

        lane = i % 2
        # Per-pass contiguous-run filtering (mirrors analyze.select_fit_window).
        for name, n in splits:
            chunk_valid = valid_all[offset:offset + n]
            chunk_theta = theta_all[offset:offset + n]
            in_win = np.array([
                analyze._in_fit_window(float(t), i, lane)
                for t in chunk_theta])
            keep_chunk = chunk_valid & in_win
            # Only keep the longest contiguous True run.
            mask_list = keep_chunk.tolist()
            s, e = analyze._longest_contiguous_run(mask_list)
            filtered = np.zeros(n, dtype=bool)
            filtered[s:e] = keep_chunk[s:e]
            out[i][name] = filtered
            offset += n
    return out


def lidar_color(i: int) -> str:
    return [
        "#1f77b4", "#ff7f0e", "#2ca02c",
        "#d62728", "#9467bd", "#8c564b",
    ][i]


def lidar_label(i: int) -> str:
    module = i // 2
    lane = i % 2
    side = ["Left", "Back", "Right"][module]
    return f"L{i} (mod{module}/{side}, lane{lane})"


def fig_polar_overlay(passes, partic, cal_x_est: float = 300.0) -> go.Figure:
    fig = make_subplots(
        rows=2, cols=3,
        specs=[[{"type": "polar"}] * 3] * 2,
        subplot_titles=[lidar_label(i) for i in range(6)],
        horizontal_spacing=0.05,
        vertical_spacing=0.10,
    )
    for i in range(6):
        row = i // 3 + 1
        col = i % 3 + 1
        for pass_name, symbol in (("fwd", "circle"), ("rev", "x")):
            if pass_name not in passes:
                continue
            p = passes[pass_name]
            d = p["d"][:, i]
            theta = p["theta"]
            valid = valid_mask(d)
            partic_m = partic[i].get(pass_name, np.zeros_like(valid))
            non_partic = valid & ~partic_m
            for sub_mask, sub_alpha, suffix in (
                (partic_m, 1.0, ""),
                (non_partic, DIM_ALPHA, " (out of fit)"),
            ):
                if not sub_mask.any():
                    continue
                fig.add_trace(
                    go.Scatterpolar(
                        r=d[sub_mask],
                        theta=np.degrees(theta[sub_mask]),
                        mode="markers",
                        marker=dict(symbol=symbol, size=4,
                                    color=lidar_color(i),
                                    opacity=sub_alpha),
                        name=f"{pass_name}{suffix}",
                        legendgroup=f"{pass_name}{suffix}",
                        showlegend=(i == 0),
                    ),
                    row=row, col=col,
                )
        # Mark the wall-perpendicular distance for reference.
        fig.add_trace(
            go.Scatterpolar(
                r=[0, cal_x_est, MAX_VALID_MM],
                theta=[0, 0, 0],
                mode="lines",
                line=dict(color="rgba(0,0,0,0.2)", dash="dot"),
                showlegend=False,
            ),
            row=row, col=col,
        )

    # Compute per-subplot polar axis index (1, 2, … not 'polar', 'polar2').
    layout_updates = {}
    for k in range(1, 7):
        key = "polar" if k == 1 else f"polar{k}"
        layout_updates[key] = dict(
            radialaxis=dict(range=[0, MAX_VALID_MM], dtick=200,
                            angle=0, gridcolor="rgba(0,0,0,0.1)"),
            angularaxis=dict(rotation=90, direction="counterclockwise",
                             gridcolor="rgba(0,0,0,0.1)"),
        )
    fig.update_layout(
        title="Polar overlay — distance vs heading per lidar (fwd vs rev)",
        height=720,
        legend=dict(orientation="h", y=-0.05),
        **layout_updates,
    )
    return fig


def fig_dist_vs_theta(passes, partic, cal_x_est: float = 300.0) -> go.Figure:
    fig = go.Figure()
    for i in range(6):
        for pass_name, dash, base_alpha in (("fwd", "solid", 1.0),
                                            ("rev", "dot", 0.45)):
            if pass_name not in passes:
                continue
            p = passes[pass_name]
            d = p["d"][:, i]
            theta = p["theta"]
            valid = valid_mask(d)
            partic_m = partic[i].get(pass_name, np.zeros_like(valid))
            non_partic = valid & ~partic_m
            sort_line = pass_name == "fwd"
            for sub_mask, alpha_mul, suffix, show in (
                (partic_m, 1.0, "", True),
                (non_partic, DIM_ALPHA, " (out of fit)", False),
            ):
                if not sub_mask.any():
                    continue
                xs = np.degrees(theta[sub_mask])
                ys = d[sub_mask]
                if sort_line:
                    order = np.argsort(xs)
                    xs = xs[order]
                    ys = ys[order]
                opacity = base_alpha * alpha_mul
                fig.add_trace(go.Scatter(
                    x=xs, y=ys,
                    mode="lines+markers" if sort_line else "markers",
                    name=f"{lidar_label(i)} · {pass_name}{suffix}",
                    legendgroup=lidar_label(i),
                    line=dict(color=lidar_color(i), dash=dash),
                    marker=dict(color=lidar_color(i), size=4,
                                opacity=opacity),
                    opacity=opacity,
                    showlegend=show,
                ))
    fig.add_hline(y=cal_x_est, line=dict(color="rgba(0,0,0,0.4)", dash="dash"),
                  annotation_text=f"CAL_X ≈ {cal_x_est:.0f} mm (estimated)",
                  annotation_position="top right")
    fig.update_layout(
        title="Distance vs heading per lidar",
        xaxis_title="asserv heading (deg)",
        yaxis_title="reading (mm)",
        height=520,
        hovermode="x unified",
    )
    return fig


def fig_sq_vs_theta(passes, partic) -> go.Figure:
    fig = go.Figure()
    for i in range(6):
        for pass_name, dash, base_alpha in (("fwd", "solid", 1.0),
                                            ("rev", "dot", 0.4)):
            if pass_name not in passes:
                continue
            p = passes[pass_name]
            sq = p["sq"][:, i]
            theta = p["theta"]
            partic_m = partic[i].get(pass_name, np.zeros(len(theta), dtype=bool))
            non_partic = ~partic_m
            sort_line = pass_name == "fwd"
            visible = True if pass_name == "fwd" else "legendonly"
            for sub_mask, alpha_mul, suffix, show in (
                (partic_m, 1.0, "", True),
                (non_partic, DIM_ALPHA, " (out of fit)", False),
            ):
                if not sub_mask.any():
                    continue
                xs = np.degrees(theta[sub_mask])
                ys = sq[sub_mask]
                if sort_line:
                    order = np.argsort(xs)
                    xs = xs[order]
                    ys = ys[order]
                opacity = base_alpha * alpha_mul
                fig.add_trace(go.Scatter(
                    x=xs, y=ys,
                    mode="lines" if sort_line else "markers",
                    name=f"{lidar_label(i)} · {pass_name}{suffix}",
                    legendgroup=lidar_label(i),
                    line=dict(color=lidar_color(i), dash=dash),
                    marker=dict(color=lidar_color(i), size=3,
                                opacity=opacity),
                    opacity=opacity,
                    visible=visible,
                    showlegend=show,
                ))
    fig.update_layout(
        title="Signal quality (sq) vs heading per lidar",
        xaxis_title="asserv heading (deg)",
        yaxis_title="sq (raw)",
        height=420,
        hovermode="x unified",
    )
    return fig


def fig_rev_repeatability(passes, partic) -> go.Figure:
    """Per-stop stdev of the 5 rev samples → noise floor estimate.
    Splits each lidar's stops into "in fit window" (full opacity,
    contributes to LSQ noise floor) and "out of fit window" (dimmed,
    surfaces off-fixture bimodality)."""
    if "rev" not in passes:
        fig = go.Figure()
        fig.update_layout(title="Reverse-pass repeatability — no rev data")
        return fig, []
    p = passes["rev"]
    theta = p["theta"]
    d = p["d"]
    step = math.radians(5.0)
    stop_idx = np.round(theta / step).astype(int)
    unique_stops, inv = np.unique(stop_idx, return_inverse=True)

    fig = go.Figure()
    summary_rows = []
    for i in range(6):
        di = d[:, i].copy()
        di[~valid_mask(di)] = np.nan
        partic_rev = partic[i].get("rev", np.zeros_like(d[:, i], dtype=bool))

        stds = np.full(len(unique_stops), np.nan)
        means = np.full(len(unique_stops), np.nan)
        counts = np.zeros(len(unique_stops), dtype=int)
        in_fit = np.zeros(len(unique_stops), dtype=bool)
        for j in range(len(unique_stops)):
            stop_mask = inv == j
            sel = di[stop_mask]
            sel = sel[~np.isnan(sel)]
            if len(sel) >= 2:
                stds[j] = np.std(sel, ddof=0)
                means[j] = np.mean(sel)
                counts[j] = len(sel)
                # The stop counts as "in fit" if any of its samples
                # ended up in the LSQ window — usually all-or-nothing.
                in_fit[j] = bool(partic_rev[stop_mask].any())

        valid = ~np.isnan(stds)
        if not valid.any():
            continue
        for sub_mask, alpha, suffix, show in (
            (valid & in_fit, 1.0, "", True),
            (valid & ~in_fit, DIM_ALPHA, " (out of fit)", False),
        ):
            if not sub_mask.any():
                continue
            xs = unique_stops[sub_mask] * 5.0
            fig.add_trace(go.Scatter(
                x=xs, y=stds[sub_mask], mode="lines+markers",
                name=f"{lidar_label(i)}{suffix}",
                legendgroup=lidar_label(i),
                line=dict(color=lidar_color(i)),
                marker=dict(color=lidar_color(i), size=5, opacity=alpha),
                opacity=alpha,
                showlegend=show,
                customdata=np.stack(
                    [means[sub_mask], counts[sub_mask]], axis=-1),
                hovertemplate=("θ=%{x:.1f}°<br>"
                               "stdev=%{y:.2f} mm<br>"
                               "mean=%{customdata[0]:.1f} mm<br>"
                               "n=%{customdata[1]}<extra>"
                               f"{lidar_label(i)}{suffix}</extra>"),
            ))

        in_fit_valid = valid & in_fit
        if in_fit_valid.any():
            summary_rows.append((
                i,
                float(np.nanmedian(stds[in_fit_valid])),
                float(np.nanmax(stds[in_fit_valid])),
                int(counts[in_fit_valid].sum()),
            ))
        else:
            summary_rows.append((i, float("nan"), float("nan"), 0))

    fig.update_layout(
        title=("Reverse-pass repeatability — stdev of the 5 samples "
               "per stop (dimmed = stop outside the fit window)"),
        xaxis_title="commanded heading (deg)",
        yaxis_title="stdev (mm)",
        height=420,
        hovermode="x unified",
    )
    return fig, summary_rows


def _wrap_pi(x: float) -> float:
    return (x + math.pi) % (2 * math.pi) - math.pi


# Physical bracket pairs (chosen by θ_min clustering, not CAN module
# index). Each entry is (lidar_a, lidar_b, label).
BRACKET_PAIRS: list[tuple[int, int, str]] = [
    (2, 3, "back (≈0°)"),
    (0, 5, "≈120°"),
    (1, 4, "≈240°"),
]


def compute_dual_fit(passes, fits, partic):
    """For each physical bracket pair, return per-sample data where
    both lidars on the bracket fall inside their fit windows. Use
    their two hit points to compute the wall plane (same math as the
    strat's `get_plane_offset`) and recover the robot heading and
    perpendicular distance, then compare to the asserv-measured
    heading."""
    out: dict[str, dict] = {}
    for i, j, name in BRACKET_PAIRS:
        if fits[i] is None or fits[j] is None:
            continue
        x_i, y_i, t_i, _, _, _ = fits[i]
        x_j, y_j, t_j, _, _, _ = fits[j]
        meas: list[float] = []
        rec: list[float] = []
        dist: list[float] = []
        pass_tags: list[str] = []
        for pn in ("fwd", "rev"):
            if pn not in passes:
                continue
            p = passes[pn]
            theta = p["theta"]
            d_i = p["d"][:, i]
            d_j = p["d"][:, j]
            pi = partic[i].get(pn, np.zeros(len(theta), dtype=bool))
            pj = partic[j].get(pn, np.zeros(len(theta), dtype=bool))
            both = pi & pj
            for k in np.flatnonzero(both):
                p0x = x_i + d_i[k] * math.cos(t_i)
                p0y = y_i + d_i[k] * math.sin(t_i)
                p1x = x_j + d_j[k] * math.cos(t_j)
                p1y = y_j + d_j[k] * math.sin(t_j)
                dx = p1x - p0x
                dy = p1y - p0y
                length = math.hypot(dx, dy)
                if length < 1e-6:
                    continue
                nx = -dy / length
                ny = dx / length
                dd = nx * p0x + ny * p0y
                if dd < 0:
                    nx, ny, dd = -nx, -ny, -dd
                angle_robot = math.atan2(ny, nx)
                # Wall at world x=0 with normal pointing outward (toward
                # the robot side at +x). The lidar pair gives that
                # normal in robot frame, which equals (π − θ_R) when
                # the robot is at world (CAL_X, 0).
                theta_rec = _wrap_pi(math.pi - angle_robot)
                meas.append(float(theta[k]))
                rec.append(theta_rec)
                dist.append(dd)
                pass_tags.append(pn)
        out[name] = {
            "pair": (i, j),
            "theta_meas": np.array(meas),
            "theta_rec": np.array(rec),
            "dist": np.array(dist),
            "pass": pass_tags,
        }
    return out


def fig_dual_fit_residuals(dual_fit, cal_x_est: float = 300.0) -> go.Figure:
    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=True,
        subplot_titles=(
            "Heading residual: wrap_pi(recovered − measured)",
            "Recovered perpendicular wall distance",
        ),
        vertical_spacing=0.12,
    )
    bracket_colors = ["#1f77b4", "#2ca02c", "#d62728"]
    for color, (name, data) in zip(bracket_colors, dual_fit.items()):
        if len(data["theta_meas"]) == 0:
            continue
        meas = data["theta_meas"]
        rec = data["theta_rec"]
        resid = np.array([_wrap_pi(r - m) for r, m in zip(rec, meas)])
        i, j = data["pair"]
        legend_name = f"{name} (L{i}+L{j})"
        fig.add_trace(go.Scatter(
            x=np.degrees(meas), y=np.degrees(resid),
            mode="markers", name=legend_name,
            marker=dict(color=color, size=5),
            legendgroup=name,
            hovertemplate=("θ_meas=%{x:.2f}°<br>"
                           "residual=%{y:.3f}°<extra>"
                           f"{legend_name}</extra>"),
        ), row=1, col=1)
        fig.add_trace(go.Scatter(
            x=np.degrees(meas), y=data["dist"],
            mode="markers", name=legend_name,
            marker=dict(color=color, size=5),
            legendgroup=name,
            showlegend=False,
            hovertemplate=("θ_meas=%{x:.2f}°<br>"
                           "dist=%{y:.2f} mm<extra>"
                           f"{legend_name}</extra>"),
        ), row=2, col=1)
    fig.add_hline(y=0, line=dict(color="rgba(0,0,0,0.3)", dash="dot"),
                  row=1, col=1)
    fig.add_hline(y=cal_x_est, line=dict(color="rgba(0,0,0,0.3)", dash="dot"),
                  row=2, col=1)
    fig.update_layout(
        title=("Calibration self-consistency — for each sample where "
               "both lidars of a physical bracket are in their fit "
               "windows, recover (θ_R, dist) and compare to asserv"),
        height=640,
        hovermode="x unified",
    )
    fig.update_xaxes(title_text="asserv heading θ_R (deg)", row=2, col=1)
    fig.update_yaxes(title_text="residual (deg)", row=1, col=1)
    fig.update_yaxes(title_text="distance (mm)", row=2, col=1)
    return fig


def apply_drift_correction(passes, top_points):
    """Apply LD06-based drift correction to raw distances.

    Returns (corrected_passes, drift_info) where *drift_info* is a dict
    with slope/total_drift/std or None if no LD06 data or fit failed.
    """
    if not top_points:
        return passes, None

    wall_dists = analyze.extract_wall_distance_from_scan(top_points)
    top_drift = analyze.fit_top_drift(wall_dists) if wall_dists else None
    if top_drift is None:
        return passes, None

    d0, a, b, slope, rss = top_drift

    # Concatenate all passes.
    thetas_list: list[np.ndarray] = []
    ds_list: list[np.ndarray] = []
    for name in ("fwd", "rev"):
        if name in passes:
            thetas_list.append(passes[name]["theta"])
            ds_list.append(passes[name]["d"])
    theta_all = np.concatenate(thetas_list)
    d_all = np.concatenate(ds_list, axis=0)

    # Build sample → step_idx mapping (fwd: 1 per step, rev: 5 per step).
    n_fwd = len(passes["fwd"]["theta"]) if "fwd" in passes else 0
    sample_to_step = list(range(n_fwd))
    n_rev = len(passes["rev"]["theta"]) if "rev" in passes else 0
    rev_step, rev_count = n_fwd, 0
    for _ in range(n_rev):
        sample_to_step.append(rev_step)
        rev_count += 1
        if rev_count % 5 == 0:
            rev_step += 1

    # Pass 1: uncorrected fits to get θ_L and CAL_X.
    cal_xs: list[float] = []
    for i in range(6):
        col = d_all[:, i]
        mask = (col >= MIN_VALID_MM) & (col <= MAX_VALID_MM)
        pts = list(zip(theta_all[mask].tolist(), col[mask].tolist()))
        fit = analyze.fit_lidar(pts, lane=i % 2, lidar_idx=i)
        if fit is not None:
            cal_xs.append(fit[3])
    cal_x_p1 = float(np.median(cal_xs)) if cal_xs else 300.0

    fits_prelim = []
    for i in range(6):
        col = d_all[:, i]
        mask = (col >= MIN_VALID_MM) & (col <= MAX_VALID_MM)
        pts = list(zip(theta_all[mask].tolist(), col[mask].tolist()))
        fits_prelim.append(analyze.fit_lidar(pts, lane=i % 2, lidar_idx=i,
                                             cal_x=cal_x_p1))

    # Correct raw distances: r_corr = r + δ / cos(θ + θ_L).
    d_corrected = d_all.copy()
    for i in range(6):
        fit = fits_prelim[i]
        if fit is None:
            continue
        _, _, theta_l, _, _, _ = fit
        for k in range(len(theta_all)):
            if d_all[k, i] < MIN_VALID_MM or d_all[k, i] > MAX_VALID_MM:
                continue
            step = sample_to_step[k] if k < len(sample_to_step) else 0
            delta = slope * step
            denom = math.cos(theta_all[k] + theta_l)
            if abs(denom) > 1e-6:
                d_corrected[k, i] = d_all[k, i] + delta / denom

    # Rebuild passes dict with corrected distances.
    corrected_passes: dict[str, dict[str, np.ndarray]] = {}
    offset = 0
    for name in ("fwd", "rev"):
        if name not in passes:
            continue
        n = len(passes[name]["theta"])
        corrected_passes[name] = {
            "theta": passes[name]["theta"].copy(),
            "d": d_corrected[offset:offset + n].copy(),
            "sq": passes[name]["sq"].copy(),
        }
        offset += n

    step_range = sample_to_step[-1] - sample_to_step[0] if sample_to_step else 0
    drift_info = {
        "slope": slope,
        "total_drift": slope * step_range,
        "n_scans": len(wall_dists),
        "ld06_offset": (a, b),
    }
    return corrected_passes, drift_info


def compute_fits(passes):
    """Run the LSQ fit (delegated to analyze.fit_lidar) on the union
    of fwd+rev samples. Returns a list of 6 fits — same tuple as
    analyze.fit_lidar — or None when the fit failed."""
    thetas: list[np.ndarray] = []
    ds: list[np.ndarray] = []
    for name in ("fwd", "rev"):
        if name in passes:
            thetas.append(passes[name]["theta"])
            ds.append(passes[name]["d"])
    theta_all = np.concatenate(thetas) if thetas else np.zeros(0)
    d_all = np.concatenate(ds, axis=0) if ds else np.zeros((0, 6))

    fits = []
    for i in range(6):
        col = d_all[:, i]
        mask = (col >= MIN_VALID_MM) & (col <= MAX_VALID_MM)
        pts = list(zip(theta_all[mask].tolist(), col[mask].tolist()))
        fits.append(analyze.fit_lidar(pts, lane=i % 2, lidar_idx=i))
    return fits


def fig_robot_layout(fits) -> go.Figure:
    """Top-down robot-frame layout: each lidar as a dot at its fitted
    (x, y), with an arrow showing its pointing direction (theta)."""
    fig = go.Figure()

    valid = [f for f in fits if f is not None]
    if valid:
        max_r = max(math.hypot(f[0], f[1]) for f in valid if f is not None)
    else:
        max_r = 200.0
    chassis_r = max_r * 1.10
    plot_extent = max(chassis_r * 1.25, 300.0)

    # Chassis envelope (circle through the outermost lidar, +10%).
    angs = np.linspace(0, 2 * math.pi, 120)
    fig.add_trace(go.Scatter(
        x=chassis_r * np.cos(angs),
        y=chassis_r * np.sin(angs),
        mode="lines",
        line=dict(color="rgba(0,0,0,0.25)", dash="dash", width=1),
        name="chassis envelope",
        hoverinfo="skip",
    ))

    # Robot center cross + +x indicator.
    fig.add_trace(go.Scatter(
        x=[0], y=[0],
        mode="markers",
        marker=dict(color="black", size=10, symbol="cross-thin",
                    line=dict(width=2)),
        name="robot center",
        hoverinfo="skip",
    ))
    fig.add_annotation(
        x=chassis_r * 1.05, y=0, ax=0, ay=0,
        xref="x", yref="y", axref="x", ayref="y",
        showarrow=True, arrowhead=2, arrowsize=1.4, arrowwidth=2,
        arrowcolor="rgba(0,0,0,0.45)",
    )
    fig.add_annotation(
        x=chassis_r * 1.10, y=0, text="+x (front)", showarrow=False,
        font=dict(color="rgba(0,0,0,0.65)", size=12),
        xanchor="left",
    )

    # Per-lidar position + orientation arrow + ray showing where the
    # beam hits when r = r_min.
    arrow_len = max(60.0, max_r * 0.35)
    for i, fit in enumerate(fits):
        if fit is None:
            continue
        x, y, theta, cal_x, r_min, th_min = fit
        color = lidar_color(i)
        dx = arrow_len * math.cos(theta)
        dy = arrow_len * math.sin(theta)

        fig.add_trace(go.Scatter(
            x=[x], y=[y], mode="markers+text",
            marker=dict(color=color, size=14, symbol="circle",
                        line=dict(color="white", width=1.5)),
            text=[f"L{i}"],
            textposition="top center",
            textfont=dict(color=color, size=12),
            name=lidar_label(i),
            hovertemplate=(f"<b>{lidar_label(i)}</b><br>"
                           f"x={x:.2f} mm<br>"
                           f"y={y:.2f} mm<br>"
                           f"θ={math.degrees(theta):.2f}°<br>"
                           f"CAL_X={cal_x:.1f} mm<br>"
                           f"r_min={r_min:.0f} mm @ θ_R={math.degrees(th_min):.1f}°"
                           "<extra></extra>"),
        ))
        fig.add_annotation(
            x=x + dx, y=y + dy, ax=x, ay=y,
            xref="x", yref="y", axref="x", ayref="y",
            showarrow=True, arrowhead=2, arrowsize=1.2, arrowwidth=2.5,
            arrowcolor=color,
        )

    fig.update_layout(
        title=("Robot top-down view (robot frame) — fitted lidar "
               "positions and pointing directions"),
        xaxis=dict(title="x (mm)", range=[-plot_extent, plot_extent],
                   zeroline=True, zerolinecolor="rgba(0,0,0,0.15)"),
        yaxis=dict(title="y (mm)", range=[-plot_extent, plot_extent],
                   zeroline=True, zerolinecolor="rgba(0,0,0,0.15)"),
        height=680,
        hovermode="closest",
        legend=dict(orientation="v", x=1.02, y=1.0),
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    return fig


def fig_xy_scatter(passes, partic, cal_x_est: float = 300.0) -> go.Figure:
    """Hit-points in robot frame assuming a point lidar at (0,0).

    Without theta_L this is just `(d cos θ, d sin θ)` — useful as a
    visual sanity check that the scan looks like a wall offset by
    ~CAL_X_MM, even before the LSQ recovers each lidar's pose.
    """
    fig = go.Figure()
    for pass_name, base_alpha in (("fwd", 1.0), ("rev", 0.35)):
        if pass_name not in passes:
            continue
        p = passes[pass_name]
        for i in range(6):
            d = p["d"][:, i]
            theta = p["theta"]
            valid = valid_mask(d)
            partic_m = partic[i].get(pass_name, np.zeros_like(valid))
            non_partic = valid & ~partic_m
            visible = True if pass_name == "fwd" else "legendonly"
            for sub_mask, alpha_mul, suffix, show in (
                (partic_m, 1.0, "", True),
                (non_partic, DIM_ALPHA, " (out of fit)", False),
            ):
                if not sub_mask.any():
                    continue
                xs = d[sub_mask] * np.cos(theta[sub_mask])
                ys = d[sub_mask] * np.sin(theta[sub_mask])
                opacity = base_alpha * alpha_mul
                fig.add_trace(go.Scatter(
                    x=xs, y=ys, mode="markers",
                    name=f"{lidar_label(i)} · {pass_name}{suffix}",
                    legendgroup=lidar_label(i),
                    marker=dict(color=lidar_color(i), size=4,
                                symbol="circle" if pass_name == "fwd" else "x",
                                opacity=opacity),
                    visible=visible,
                    showlegend=show,
                ))
    # Reference: the wall is at x = -CAL_X relative to the robot when
    # heading 0, so plotting in the *world* frame would show it as a
    # vertical line at x = -CAL_X. Here we use `d * (cosθ, sinθ)` which
    # is wall-tangent-rotating-with-the-robot, so the hit-points trace
    # the full circle — useful to spot dead spots / clutter angles.
    fig.add_shape(type="circle", xref="x", yref="y",
                  x0=-cal_x_est, y0=-cal_x_est, x1=cal_x_est, y1=cal_x_est,
                  line=dict(color="rgba(0,0,0,0.2)", dash="dot"))
    fig.update_layout(
        title="Hit-points in robot frame (ignoring lidar pose) — "
              "wall should trace a circle of radius ≈ CAL_X",
        xaxis_title="x (mm) = d·cos θ",
        yaxis_title="y (mm) = d·sin θ",
        height=620,
        hovermode="closest",
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    return fig


def render(passes, summary, in_path: Path, out_path: Path):
    figs = [
        fig_polar_overlay(passes),
        fig_dist_vs_theta(passes),
        fig_xy_scatter(passes),
        fig_sq_vs_theta(passes),
    ]

    # Build summary table HTML.
    rows = ""
    for name in ("fwd", "rev"):
        if name not in passes:
            continue
        p = passes[name]
        n = len(p["theta"])
        for i in range(6):
            valid = valid_mask(p["d"][:, i]).sum()
            rows += (f"<tr><td>{name}</td><td>{lidar_label(i)}</td>"
                     f"<td>{n}</td><td>{valid}</td>"
                     f"<td>{int(np.median(p['sq'][:, i][valid_mask(p['d'][:, i])])) if valid else '-'}</td>"
                     f"</tr>")

    rep_rows = ""
    for i, med, mx, total in summary:
        med_s = f"{med:.2f}" if med == med else "-"  # NaN check
        mx_s = f"{mx:.2f}" if mx == mx else "-"
        rep_rows += (f"<tr><td>{lidar_label(i)}</td>"
                     f"<td>{med_s}</td><td>{mx_s}</td>"
                     f"<td>{total}</td></tr>")

    plot_divs = "\n".join(
        f.to_html(full_html=False, include_plotlyjs=("cdn" if k == 0 else False))
        for k, f in enumerate(figs)
    )

    html = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Galipeur ground-lidar calibration sweep</title>
<style>
  body {{ font-family: system-ui, -apple-system, Segoe UI, sans-serif;
          margin: 24px; color: #222; }}
  h1 {{ margin-bottom: 4px; }}
  .meta {{ color: #666; font-size: 14px; margin-bottom: 16px; }}
  section {{ margin: 24px 0; }}
  table {{ border-collapse: collapse; font-size: 14px; }}
  th, td {{ border: 1px solid #ddd; padding: 4px 10px; text-align: right; }}
  th {{ background: #f5f5f5; text-align: left; }}
  td:first-child, th:first-child {{ text-align: left; }}
  .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 24px; }}
  @media (max-width: 1100px) {{ .grid {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<h1>Galipeur ground-lidar calibration sweep</h1>
<div class="meta">Source: <code>{in_path}</code> ·
  CAL_X ≈ {cal_x_est:.1f} mm (estimated) ·
  valid window: [{MIN_VALID_MM:.0f}, {MAX_VALID_MM}] mm
</div>

<div class="grid">
  <section>
    <h2>Coverage</h2>
    <table>
      <tr><th>pass</th><th>lidar</th><th>n samples</th>
          <th>n valid</th><th>median sq (valid)</th></tr>
      {rows}
    </table>
  </section>

  <section>
    <h2>Rev-pass repeatability (stops in fit window only)</h2>
    <table>
      <tr><th>lidar</th><th>median stdev (mm)</th>
          <th>max stdev (mm)</th><th>n samples</th></tr>
      {rep_rows}
    </table>
  </section>
</div>

{plot_divs}

</body>
</html>
"""
    out_path.write_text(html)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("file", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None)
    args = ap.parse_args()

    in_path = args.file
    out_path = args.out or in_path.with_suffix(".html")

    passes = parse(in_path)
    if not passes:
        print(f"no samples found between BEGIN/END markers in {in_path}",
              file=sys.stderr)
        return 1

    # Parse LD06 data for drift correction.
    with open(in_path) as f:
        _, top_points = analyze.parse(f)
    corrected_passes, drift_info = apply_drift_correction(passes, top_points)
    if drift_info is not None:
        print(f"LD06 drift correction: slope={drift_info['slope']:.3f} mm/step, "
              f"total={drift_info['total_drift']:+.1f} mm "
              f"({drift_info['n_scans']} scans)", file=sys.stderr)

    fits = compute_fits(corrected_passes)
    partic = participating_masks(corrected_passes)
    cal_xs = [f[3] for f in fits if f is not None]
    cal_x_est = sum(cal_xs) / len(cal_xs) if cal_xs else 300.0
    _rep_fig, summary = fig_rev_repeatability(corrected_passes, partic)
    dual_fit = compute_dual_fit(corrected_passes, fits, partic)
    # Render full doc — robot layout first since it's the headline.
    figs_main = [
        fig_robot_layout(fits),
        fig_polar_overlay(corrected_passes, partic, cal_x_est),
        fig_dist_vs_theta(corrected_passes, partic, cal_x_est),
        fig_xy_scatter(corrected_passes, partic, cal_x_est),
        fig_sq_vs_theta(corrected_passes, partic),
        _rep_fig,
        fig_dual_fit_residuals(dual_fit, cal_x_est),
    ]

    # Build coverage table (use raw passes for sample counts).
    cov_rows = ""
    for name in ("fwd", "rev"):
        if name not in passes:
            continue
        p = passes[name]
        n = len(p["theta"])
        for i in range(6):
            v_mask = valid_mask(p["d"][:, i])
            valid = int(v_mask.sum())
            med_sq = int(np.median(p["sq"][:, i][v_mask])) if valid else "-"
            cov_rows += (f"<tr><td>{name}</td><td>{lidar_label(i)}</td>"
                         f"<td>{n}</td><td>{valid}</td>"
                         f"<td>{med_sq}</td></tr>")

    rep_rows = ""
    for i, med, mx, total in summary:
        med_s = f"{med:.2f}" if med == med else "-"  # NaN check
        mx_s = f"{mx:.2f}" if mx == mx else "-"
        rep_rows += (f"<tr><td>{lidar_label(i)}</td>"
                     f"<td>{med_s}</td><td>{mx_s}</td>"
                     f"<td>{total}</td></tr>")

    dual_rows = ""
    for name, data in dual_fit.items():
        if len(data["theta_meas"]) == 0:
            i, j = data["pair"]
            dual_rows += (f"<tr><td>{name}</td><td>L{i}+L{j}</td>"
                          "<td colspan=4 style='text-align:center'>"
                          "no overlap of fit windows</td></tr>")
            continue
        meas = data["theta_meas"]
        rec = data["theta_rec"]
        resid = np.array([_wrap_pi(r - m) for r, m in zip(rec, meas)])
        i, j = data["pair"]
        dual_rows += (
            f"<tr><td>{name}</td><td>L{i}+L{j}</td>"
            f"<td>{len(meas)}</td>"
            f"<td>{math.degrees(np.mean(resid)):+.3f}</td>"
            f"<td>{math.degrees(np.std(resid)):.3f}</td>"
            f"<td>{np.mean(data['dist']):.2f} ± {np.std(data['dist']):.2f}</td>"
            "</tr>"
        )

    fit_rows = ""
    for i, fit in enumerate(fits):
        if fit is None:
            fit_rows += (f"<tr><td>{lidar_label(i)}</td>"
                         "<td colspan=6 style='text-align:center'>"
                         "FIT FAILED</td></tr>")
            continue
        x, y, theta, cal_x, r_min, th_min = fit
        fit_rows += (
            f"<tr><td>{lidar_label(i)}</td>"
            f"<td>{x:.2f}</td><td>{y:.2f}</td>"
            f"<td>{math.degrees(theta):.2f}</td>"
            f"<td>{cal_x:.1f}</td>"
            f"<td>{r_min:.1f}</td>"
            f"<td>{math.degrees(th_min):.1f}</td></tr>"
        )

    # Generate paste-ready Rust code block.
    conf_lines = ["GroundLidarConf {", "    modules: ["]
    for m in range(3):
        conf_lines.append(f"        // Module {m}: lidar {m * 2} and lidar {m * 2 + 1}")
        conf_lines.append("        [")
        for l in range(2):
            idx = m * 2 + l
            fit = fits[idx]
            if fit is None:
                conf_lines.append(
                    f"            GroundLidarPose {{ x: 0.0, y: 0.0, theta: 0.0 }},  "
                    f"// lidar {idx}: FIT FAILED")
            else:
                x, y, t, cal_x, r_min, th_min = fit
                conf_lines.append(
                    f"            GroundLidarPose {{ x: {x:.2f}, y: {y:.2f}, "
                    f"theta: {math.degrees(t):.3f}_f32.to_radians() }},  "
                    f"// r_min={r_min:.0f} mm @ θ_R={math.degrees(th_min):.1f}°")
        conf_lines.append("        ],")
    conf_lines.append("    ],")
    conf_lines.append("}")
    conf_code = "\n".join(conf_lines)

    if drift_info is not None:
        drift_info_html = (
            f"LD06 drift: {drift_info['total_drift']:+.1f} mm "
            f"({drift_info['slope']:.3f} mm/step, "
            f"{drift_info['n_scans']} scans)")
    else:
        drift_info_html = "no LD06 drift correction"

    plot_divs = "\n".join(
        f.to_html(full_html=False,
                  include_plotlyjs=("cdn" if k == 0 else False))
        for k, f in enumerate(figs_main)
    )

    html = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Galipeur ground-lidar calibration sweep</title>
<style>
  body {{ font-family: system-ui, -apple-system, Segoe UI, sans-serif;
          margin: 24px; color: #222; }}
  h1 {{ margin-bottom: 4px; }}
  .meta {{ color: #666; font-size: 14px; margin-bottom: 16px; }}
  section {{ margin: 24px 0; }}
  table {{ border-collapse: collapse; font-size: 14px; }}
  th, td {{ border: 1px solid #ddd; padding: 4px 10px; text-align: right; }}
  th {{ background: #f5f5f5; text-align: left; }}
  td:first-child, th:first-child {{ text-align: left; }}
  .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 24px; }}
  @media (max-width: 1100px) {{ .grid {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<h1>Galipeur ground-lidar calibration sweep</h1>
<div class="meta">Source: <code>{in_path}</code> ·
  CAL_X ≈ {cal_x_est:.1f} mm (estimated) ·
  valid window: [{MIN_VALID_MM:.0f}, {MAX_VALID_MM}] mm ·
  {drift_info_html}
</div>

<section>
  <h2>Fitted poses (robot frame)</h2>
  <table>
    <tr><th>lidar</th><th>x (mm)</th><th>y (mm)</th>
        <th>θ (°)</th><th>CAL_X (mm)</th><th>r_min (mm)</th><th>θ_R @ r_min (°)</th></tr>
    {fit_rows}
  </table>
</section>

<section>
  <h2>Paste into main.rs</h2>
  <pre style="background:#f5f5f5;padding:12px;border-radius:4px;font-size:13px;overflow-x:auto">{conf_code}</pre>
</section>

<section>
  <h2>Dual-lidar wall fit consistency (per physical bracket)</h2>
  <p style="font-size:13px;color:#555;margin-top:0">
    Same math as the strat's <code>get_plane_offset</code>: for each
    sample where both lidars on a physical bracket are inside their
    fit windows, recover (θ_R, dist) from the two hit points and
    compare to the asserv heading.
  </p>
  <table>
    <tr><th>bracket</th><th>pair</th><th>n samples</th>
        <th>mean residual (°)</th><th>stdev residual (°)</th>
        <th>dist (mm) — mean ± stdev</th></tr>
    {dual_rows}
  </table>
</section>

<div class="grid">
  <section>
    <h2>Coverage</h2>
    <table>
      <tr><th>pass</th><th>lidar</th><th>n samples</th>
          <th>n valid</th><th>median sq (valid)</th></tr>
      {cov_rows}
    </table>
  </section>

  <section>
    <h2>Rev-pass repeatability (stops in fit window only)</h2>
    <table>
      <tr><th>lidar</th><th>median stdev (mm)</th>
          <th>max stdev (mm)</th><th>n samples</th></tr>
      {rep_rows}
    </table>
  </section>
</div>

{plot_divs}

</body>
</html>
"""
    out_path.write_text(html)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
