#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
animate_cutin.py - Animate the cut-in vehicle following demo.

Shows two vehicles (ego=blue, lead=red) in bird's-eye view.
The lead vehicle cuts in from an adjacent lane; the ego detects it,
brakes, then follows the lead's trajectory.

Usage:
    python tools/animate_cutin.py [result_dir] [route_csv] [options]

    result_dir : directory with cutin_trace.csv  (default: build/Release)
    route_csv  : reference path CSV              (default: data/reference_route.csv)

Options:
    --step N    use every N-th simulation step   (default: 5)
    --fps  N    playback frame rate              (default: 20)
    --save FILE export to .gif (Pillow) or .mp4 (FFmpeg); omit for interactive
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------
CAR_LEN = 5.0   # [m] — slightly exaggerated for visibility
CAR_WID = 2.2

BG_DARK  = "#0d1117"
BG_PANEL = "#161b22"
COL_EGO  = "#4c9be8"
COL_LEAD = "#e84c4c"
COL_PATH = "#4a5568"
COL_ADJ  = "#fc8c03"
COL_GRID = "#8b949e"
COL_TEXT = "#e6edf3"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def car_corners(x: float, y: float, yaw: float,
                length: float = CAR_LEN, width: float = CAR_WID) -> np.ndarray:
    local = np.array([
        [ length / 2,  width / 2],
        [ length / 2, -width / 2],
        [-length / 2, -width / 2],
        [-length / 2,  width / 2],
    ])
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return (rot @ local.T).T + np.array([x, y])


def adjacent_lane(rx: np.ndarray, ry: np.ndarray,
                  lat_offset: float) -> tuple[np.ndarray, np.ndarray]:
    """Return a path parallel to (rx, ry), offset to the left by lat_offset."""
    dx = np.gradient(rx)
    dy = np.gradient(ry)
    L = np.hypot(dx, dy)
    L = np.where(L < 1e-9, 1e-9, L)
    nx = -dy / L   # left-normal x
    ny =  dx / L   # left-normal y
    return rx + nx * lat_offset, ry + ny * lat_offset


# ---------------------------------------------------------------------------
# Data loaders
# ---------------------------------------------------------------------------

def load_route(path: str) -> tuple[np.ndarray, np.ndarray]:
    xs, ys = [], []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return np.array(xs), np.array(ys)


def load_trace(path: str) -> dict[str, np.ndarray]:
    rows: list[dict] = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("result_dir", nargs="?",
                    default=os.path.join("build", "Release"))
    ap.add_argument("route_csv", nargs="?",
                    default=os.path.join("data", "reference_route.csv"))
    ap.add_argument("--step", type=int, default=5,
                    help="subsample every N-th step (default: 5)")
    ap.add_argument("--fps",  type=int, default=20,
                    help="playback fps (default: 20)")
    ap.add_argument("--save", default="",
                    help="save to .gif or .mp4; omit for interactive window")
    args = ap.parse_args()

    if not args.save:
        try:
            matplotlib.use("TkAgg")
        except Exception:
            pass

    trace_path = os.path.join(args.result_dir, "cutin_trace.csv")
    if not os.path.exists(trace_path):
        print(f"Error: {trace_path} not found. Run cutin_demo first.")
        return 1
    if not os.path.exists(args.route_csv):
        print(f"Error: route CSV not found: {args.route_csv}")
        return 1

    d = load_trace(trace_path)
    step = args.step

    t        = d["time_s"][::step]
    ex       = d["x_m"][::step]
    ey       = d["y_m"][::step]
    eyaw     = d["yaw_rad"][::step]
    ev_kmh   = d["ego_speed_kmh"][::step]
    lx       = d["lead_x_m"][::step]
    ly       = d["lead_y_m"][::step]
    lyaw     = d["lead_yaw_rad"][::step]
    lv_kmh   = d["lead_speed_kmh"][::step]
    gap      = d["gap_m"][::step]
    mode     = d["acc_mode"][::step]
    lead_lat = d["lead_lat_m"][::step]

    rx, ry = load_route(args.route_csv)

    n_frames = len(t)
    T_MAX    = float(t[-1])

    # Adjacent lane path (where lead vehicle starts)
    LATERAL_OFFSET = float(lead_lat[lead_lat > 0][0]) if np.any(lead_lat > 0) else 3.5
    adj_x, adj_y = adjacent_lane(rx, ry, LATERAL_OFFSET)

    # Cut-in frame: first where acc_mode == 1
    cutin_idx = int(np.where(mode == 1.0)[0][0]) if np.any(mode == 1.0) else None
    cutin_t   = float(t[cutin_idx]) if cutin_idx is not None else None

    # First frame where lead is active (gap > 0)
    lead_start_idx = int(np.where(gap > 0)[0][0]) if np.any(gap > 0) else None

    # Desired gap curve
    ev_mps  = ev_kmh / 3.6
    gap_des = np.maximum(5.0, 2.0 * ev_mps)
    gap_vis = np.where(gap > 0, gap, np.nan)

    # ── Figure layout ─────────────────────────────────────────────────────────
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(15, 9))
    fig.patch.set_facecolor(BG_DARK)

    gs     = fig.add_gridspec(3, 2, hspace=0.50, wspace=0.30,
                              left=0.06, right=0.97, top=0.93, bottom=0.07)
    ax_map = fig.add_subplot(gs[:2, :])
    ax_spd = fig.add_subplot(gs[2, 0])
    ax_gap = fig.add_subplot(gs[2, 1])

    for ax in (ax_map, ax_spd, ax_gap):
        ax.set_facecolor(BG_PANEL)
        ax.grid(True, alpha=0.12, color=COL_GRID, linewidth=0.5)
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")
        ax.tick_params(colors=COL_GRID)

    # ── Map ───────────────────────────────────────────────────────────────────
    ax_map.plot(rx, ry, "--", color=COL_PATH, linewidth=1.5, alpha=0.8,
                label="Reference path (ego lane)")
    ax_map.plot(adj_x, adj_y, ":", color=COL_ADJ, linewidth=1.0, alpha=0.55,
                label="Adjacent lane (lead start)")
    ax_map.plot(rx[0],  ry[0],  "o", color="#3fb950", markersize=9,
                zorder=7, label="Start", markeredgecolor="white", markeredgewidth=0.8)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("x [m]", color=COL_GRID)
    ax_map.set_ylabel("y [m]", color=COL_GRID)
    ax_map.set_title("Cut-in Following Demo  —  ego (blue) tracks lead (red) after lane change",
                     color=COL_TEXT, fontsize=12, fontweight="bold", pad=10)

    # Trajectory tails
    ego_tail,  = ax_map.plot([], [], color=COL_EGO,  linewidth=2.0, alpha=0.75,
                              label="Ego trajectory", zorder=3)
    lead_tail, = ax_map.plot([], [], color=COL_LEAD, linewidth=2.0, alpha=0.75,
                              label="Lead trajectory", zorder=3)

    # Vehicle patches (ego always visible; lead starts hidden)
    ego_patch = mpatches.Polygon(
        car_corners(ex[0], ey[0], eyaw[0]), closed=True,
        facecolor=COL_EGO, edgecolor="white", linewidth=0.9, alpha=0.95, zorder=6)
    ax_map.add_patch(ego_patch)

    lead_patch = mpatches.Polygon(
        car_corners(ex[0], ey[0], 0.0), closed=True,
        facecolor=COL_LEAD, edgecolor="white", linewidth=0.9, alpha=0.95, zorder=6)
    lead_patch.set_visible(False)
    ax_map.add_patch(lead_patch)

    # Direction arrows on vehicles (updated each frame)
    ego_arrow  = ax_map.annotate("", xy=(ex[0], ey[0]),
        xytext=(ex[0] - 2 * math.cos(eyaw[0]), ey[0] - 2 * math.sin(eyaw[0])),
        arrowprops=dict(arrowstyle="-|>", color="white", lw=1.0), zorder=7)
    lead_arrow = ax_map.annotate("", xy=(ex[0], ey[0]),
        xytext=(ex[0], ey[0]),
        arrowprops=dict(arrowstyle="-|>", color="white", lw=1.0), zorder=7)
    lead_arrow.set_visible(False)

    ax_map.legend(loc="upper right", fontsize=8.0,
                  facecolor=BG_PANEL, edgecolor="#30363d",
                  labelcolor=COL_TEXT, framealpha=0.9)

    time_text = ax_map.text(
        0.01, 0.97, "", transform=ax_map.transAxes,
        fontsize=10, color=COL_TEXT, va="top",
        bbox=dict(boxstyle="round,pad=0.4", facecolor=BG_PANEL,
                  edgecolor="#30363d", alpha=0.85))
    phase_text = ax_map.text(
        0.01, 0.88, "", transform=ax_map.transAxes,
        fontsize=9.5, color=COL_ADJ, va="top", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.3", facecolor=BG_PANEL,
                  edgecolor="none", alpha=0.80))

    # ── Speed subplot ─────────────────────────────────────────────────────────
    ax_spd.plot(t, ev_kmh, color=COL_EGO,  linewidth=1.8, label="Ego")
    lead_mask = lv_kmh > 0
    if np.any(lead_mask):
        ax_spd.plot(t[lead_mask], lv_kmh[lead_mask], color=COL_LEAD,
                    linewidth=1.5, linestyle="--", label="Lead")
    if cutin_t is not None:
        ax_spd.axvline(cutin_t, color=COL_ADJ, linewidth=1.2,
                       linestyle=":", alpha=0.8, label=f"Cut-in t={cutin_t:.1f}s")
    ax_spd.set_xlabel("Time [s]", color=COL_GRID)
    ax_spd.set_ylabel("Speed [km/h]", color=COL_GRID)
    ax_spd.set_title("Speed", color=COL_TEXT, fontsize=10)
    ax_spd.legend(fontsize=7.5, facecolor=BG_PANEL, edgecolor="#30363d",
                  labelcolor=COL_TEXT)
    ax_spd.set_xlim(0, T_MAX)
    ax_spd.set_ylim(bottom=0)
    spd_cursor, = ax_spd.plot([], [], color="white", linewidth=0.8, alpha=0.45, zorder=4)
    spd_dot,    = ax_spd.plot([], [], "o", color=COL_EGO, markersize=5, zorder=5,
                               markeredgecolor="white", markeredgewidth=0.5)

    # ── Gap subplot ───────────────────────────────────────────────────────────
    ax_gap.plot(t, gap_vis, color="#3fb950", linewidth=1.8, label="Actual gap")
    ax_gap.plot(t, gap_des, color=COL_ADJ,  linewidth=1.3, linestyle="--",
                label="Desired gap (2 s headway)")
    ax_gap.axhline(5.0, color=COL_LEAD, linewidth=1.0, linestyle=":", alpha=0.7,
                   label="Min gap (5 m)")
    if cutin_t is not None:
        ax_gap.axvline(cutin_t, color=COL_ADJ, linewidth=1.2,
                       linestyle=":", alpha=0.8)
    ax_gap.set_xlabel("Time [s]", color=COL_GRID)
    ax_gap.set_ylabel("Gap [m]", color=COL_GRID)
    ax_gap.set_title("Gap to lead vehicle", color=COL_TEXT, fontsize=10)
    ax_gap.legend(fontsize=7.5, facecolor=BG_PANEL, edgecolor="#30363d",
                  labelcolor=COL_TEXT)
    ax_gap.set_xlim(0, T_MAX)
    gap_cursor, = ax_gap.plot([], [], color="white", linewidth=0.8, alpha=0.45, zorder=4)
    gap_dot,    = ax_gap.plot([], [], "o", color="#3fb950", markersize=5, zorder=5,
                               markeredgecolor="white", markeredgewidth=0.5)

    TAIL = 120   # frames kept in trajectory tail

    # ── Animation update ──────────────────────────────────────────────────────
    def update(frame: int):
        idx = min(frame, n_frames - 1)
        cur_t = float(t[idx])

        # --- Ego vehicle ---
        ego_patch.set_xy(car_corners(float(ex[idx]), float(ey[idx]), float(eyaw[idx])))
        aw = float(eyaw[idx])
        ego_arrow.set_position((float(ex[idx]) + CAR_LEN * 0.6 * math.cos(aw),
                                float(ey[idx]) + CAR_LEN * 0.6 * math.sin(aw)))
        ego_arrow.xy = (float(ex[idx]) + CAR_LEN * 0.8 * math.cos(aw),
                        float(ey[idx]) + CAR_LEN * 0.8 * math.sin(aw))

        # --- Lead vehicle (show only when active) ---
        lead_visible = lead_start_idx is not None and idx >= lead_start_idx
        lead_patch.set_visible(lead_visible)
        lead_arrow.set_visible(lead_visible)
        if lead_visible:
            lead_patch.set_xy(
                car_corners(float(lx[idx]), float(ly[idx]), float(lyaw[idx])))
            lw = float(lyaw[idx])
            lead_arrow.set_position(
                (float(lx[idx]) + CAR_LEN * 0.6 * math.cos(lw),
                 float(ly[idx]) + CAR_LEN * 0.6 * math.sin(lw)))
            lead_arrow.xy = (
                float(lx[idx]) + CAR_LEN * 0.8 * math.cos(lw),
                float(ly[idx]) + CAR_LEN * 0.8 * math.sin(lw))

        # --- Trajectory tails ---
        s0 = max(0, idx - TAIL)
        ego_tail.set_data(ex[s0:idx + 1], ey[s0:idx + 1])
        if lead_start_idx is not None and idx >= lead_start_idx:
            s1 = max(lead_start_idx, idx - TAIL)
            lead_tail.set_data(lx[s1:idx + 1], ly[s1:idx + 1])
        else:
            lead_tail.set_data([], [])

        # --- Phase label ---
        if lead_start_idx is None or idx < lead_start_idx:
            phase = "Phase 0  Free cruise"
        elif int(mode[idx]) == 0:
            lat = float(lead_lat[idx])
            if lat > 1.0:
                phase = f"Phase 1  Cut-in detected  (lat={lat:.1f} m)"
            else:
                phase = "Phase 1  Braking / adjusting"
        else:
            phase = "Phase 2  Following lead"
        phase_text.set_text(phase)

        # --- HUD ---
        lat_str = (f"  lead lat={float(lead_lat[idx]):.1f} m" if lead_visible else "")
        time_text.set_text(
            f"t = {cur_t:.1f} s    "
            f"ego {ev_kmh[idx]:.1f} km/h"
            + lat_str)

        # --- Speed cursor ---
        ylim_s = ax_spd.get_ylim()
        spd_cursor.set_data([cur_t, cur_t], list(ylim_s))
        spd_dot.set_data([cur_t], [ev_kmh[idx]])

        # --- Gap cursor ---
        ylim_g = ax_gap.get_ylim()
        gap_cursor.set_data([cur_t, cur_t], list(ylim_g))
        gv = float(gap_vis[idx])
        if np.isfinite(gv):
            gap_dot.set_data([cur_t], [gv])
        else:
            gap_dot.set_data([], [])

        return (ego_patch, lead_patch, ego_tail, lead_tail,
                time_text, phase_text,
                spd_cursor, spd_dot, gap_cursor, gap_dot)

    anim = FuncAnimation(
        fig, update, frames=n_frames,
        interval=max(1, 1000 // args.fps),
        blit=False)

    # ── Output ────────────────────────────────────────────────────────────────
    if args.save:
        ext = os.path.splitext(args.save)[1].lower()
        print(f"Saving to {args.save} ...")
        if ext == ".gif":
            try:
                from PIL import Image  # noqa: F401
            except ImportError:
                sys.exit("Pillow is required for GIF export:  pip install pillow")
            writer = PillowWriter(fps=args.fps)
        else:
            writer = FFMpegWriter(fps=args.fps, bitrate=2000,
                                  extra_args=["-pix_fmt", "yuv420p"])
        anim.save(args.save, writer=writer, dpi=110,
                  savefig_kwargs={"facecolor": fig.get_facecolor()})
        print(f"Saved: {args.save}")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
