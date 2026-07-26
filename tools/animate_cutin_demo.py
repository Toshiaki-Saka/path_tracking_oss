#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
animate_cutin_demo.py - Animate the cut-in vehicle following control demo.

Layout (dark theme):
  - Top panel  : XY map — ego vehicle (blue) + lead vehicle (red, appears at cut-in)
  - Bottom-left : Speed vs time — ego / lead / phase labels
  - Bottom-mid  : Gap to lead vs time
  - Bottom-right: Longitudinal acceleration vs time

Usage:
    python tools/animate_cutin_demo.py [result_dir] [route_csv] [options]

    result_dir : directory with cutin_trace.csv  (default: build/Release)
    route_csv  : reference path CSV              (default: data/reference_route.csv)

Options:
    --step N    subsample every N-th step  (default: 12)
    --fps  N    playback frame rate        (default: 30)
    --save FILE export to .gif or .mp4; omit for interactive window
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
# Car geometry
# ---------------------------------------------------------------------------
CAR_LEN = 7.0
CAR_WID = 3.2


def car_corners(x, y, yaw, length=CAR_LEN, width=CAR_WID):
    local = np.array([
        [ length / 2,  width / 2],
        [ length / 2, -width / 2],
        [-length / 2, -width / 2],
        [-length / 2,  width / 2],
    ])
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return (rot @ local.T).T + np.array([x, y])


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_trace(path):
    keys = ("time_s", "x_m", "y_m", "yaw_rad",
            "ego_speed_kmh", "lead_speed_kmh",
            "ego_s_m", "lead_s_m", "gap_m",
            "accel_mps2", "acc_mode")
    data = {k: [] for k in keys}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            for k in keys:
                data[k].append(float(row[k]))
    return {k: np.asarray(v) for k, v in data.items()}


def load_route(path):
    xs, ys, yaws, ss = [], [], [], []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
            try:
                yaws.append(float(row["yaw"]))
            except (KeyError, ValueError):
                yaws.append(0.0)
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    yaws = np.asarray(yaws)
    ds = np.hypot(np.diff(xs), np.diff(ys))
    ss = np.concatenate([[0.0], np.cumsum(ds)])
    return ss, xs, ys, yaws


def interp_route(ss, xs, ys, yaws, s_query):
    """Return (x, y, yaw) at arc-length s_query by linear interpolation."""
    s_query = float(np.clip(s_query, ss[0], ss[-1]))
    idx = int(np.searchsorted(ss, s_query)) - 1
    idx = max(0, min(idx, len(ss) - 2))
    frac = (s_query - ss[idx]) / max(ss[idx + 1] - ss[idx], 1e-9)
    x = xs[idx] + frac * (xs[idx + 1] - xs[idx])
    y = ys[idx] + frac * (ys[idx + 1] - ys[idx])
    # Angle wrap-safe interpolation
    a0, a1 = yaws[idx], yaws[idx + 1]
    diff = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    yaw = a0 + frac * diff
    return x, y, yaw


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("result_dir", nargs="?",
                    default=os.path.join("build", "Release"))
    ap.add_argument("route_csv",  nargs="?",
                    default=os.path.join("data", "reference_route.csv"))
    ap.add_argument("--step", type=int, default=12)
    ap.add_argument("--fps",  type=int, default=30)
    ap.add_argument("--save", default="",
                    help="save to .gif or .mp4")
    args = ap.parse_args()

    if not args.save:
        try:
            matplotlib.use("TkAgg")
        except Exception:
            pass

    # ── load data ─────────────────────────────────────────────────────────
    trace_path = os.path.join(args.result_dir, "cutin_trace.csv")
    if not os.path.exists(trace_path):
        sys.exit(f"cutin_trace.csv not found in {args.result_dir}. Run cutin_demo first.")

    d = load_trace(trace_path)
    print(f"  loaded {trace_path}  ({len(d['time_s'])} steps)")

    rs, rx, ry, ryaw = load_route(args.route_csv)
    print(f"  loaded route: {args.route_csv}")

    # Precompute lead XY at each simulation step
    lead_xy = np.array([
        interp_route(rs, rx, ry, ryaw, s) for s in d["lead_s_m"]
    ])
    lx_all  = lead_xy[:, 0]
    ly_all  = lead_xy[:, 1]
    lyaw_all = lead_xy[:, 2]

    # Desired gap (time_gap=2s, min_gap=5m)
    gap_des_all = np.maximum(5.0, 2.0 * d["ego_speed_kmh"] / 3.6)

    # Cut-in frame index
    cutin_indices = np.where(d["acc_mode"] == 1.0)[0]
    cutin_raw = int(cutin_indices[0]) if len(cutin_indices) else None
    cutin_t   = float(d["time_s"][cutin_raw]) if cutin_raw is not None else None

    # Subsample
    idx = np.arange(0, len(d["time_s"]), args.step)
    sub = {k: v[idx] for k, v in d.items()}
    sub_lx   = lx_all[idx]
    sub_ly   = ly_all[idx]
    sub_lyaw = lyaw_all[idx]
    sub_gdes = gap_des_all[idx]
    n_frames = len(idx)

    # Cut-in frame in subsampled space
    cutin_frame = None
    if cutin_raw is not None:
        cutin_frame = int(cutin_raw // args.step)

    print(f"  frames : {n_frames}  (step={args.step}, fps={args.fps})")
    print(f"  duration: ~{n_frames / args.fps:.1f} s")
    if cutin_t is not None:
        print(f"  cut-in : t={cutin_t:.1f} s  → frame {cutin_frame}")

    # ── figure ────────────────────────────────────────────────────────────
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor("#0d1117")

    gs = fig.add_gridspec(3, 3,
                          hspace=0.50, wspace=0.38,
                          left=0.06, right=0.97, top=0.92, bottom=0.07)
    ax_map = fig.add_subplot(gs[:2, :])   # top 2/3 — full width map
    ax_spd = fig.add_subplot(gs[2, 0])
    ax_gap = fig.add_subplot(gs[2, 1])
    ax_acc = fig.add_subplot(gs[2, 2])

    MAP_BG = "#161b22"
    for ax in (ax_map, ax_spd, ax_gap, ax_acc):
        ax.set_facecolor(MAP_BG)
        ax.grid(True, alpha=0.12, color="#8b949e", linewidth=0.5)
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")
        ax.tick_params(colors="#8b949e")

    # Reference route
    ax_map.plot(rx, ry, "--", color="#8b949e", linewidth=1.0, alpha=0.5,
                label="Reference path", zorder=1)
    ax_map.plot(rx[0],  ry[0],  "o", color="#3fb950", markersize=9, zorder=6,
                label="Start", markeredgecolor="white", markeredgewidth=0.8)
    ax_map.plot(rx[-1], ry[-1], "s", color="#f78166", markersize=9, zorder=6,
                label="Goal",  markeredgecolor="white", markeredgewidth=0.8)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("x [m]", color="#8b949e")
    ax_map.set_ylabel("y [m]", color="#8b949e")
    ax_map.set_title("Cut-in Following Control Demo", color="#e6edf3",
                     fontsize=13, fontweight="bold", pad=8)

    # Speed panel
    max_v = max(float(np.max(sub["ego_speed_kmh"])), 1.0)
    ax_spd.plot(sub["time_s"], sub["ego_speed_kmh"],
                color="#4c9be8", linewidth=0.8, alpha=0.35)
    lead_mask = sub["lead_speed_kmh"] > 0
    if np.any(lead_mask):
        ax_spd.plot(sub["time_s"][lead_mask], sub["lead_speed_kmh"][lead_mask],
                    color="#e84c4c", linewidth=0.8, alpha=0.35, linestyle="--")
    ax_spd.set_xlim(0, float(sub["time_s"][-1]))
    ax_spd.set_ylim(0, max_v * 1.3)
    ax_spd.set_xlabel("Time [s]", color="#8b949e", fontsize=8)
    ax_spd.set_ylabel("Speed [km/h]", color="#8b949e", fontsize=8)
    ax_spd.set_title("Speed", color="#e6edf3", fontsize=9)
    if cutin_t is not None:
        ax_spd.axvline(cutin_t, color="#f0a030", linewidth=1.0,
                       linestyle=":", alpha=0.7)

    # Gap panel
    gap_valid = np.where(sub["gap_m"] > 0, sub["gap_m"], np.nan)
    ax_gap.plot(sub["time_s"], gap_valid,
                color="#4ce87a", linewidth=0.8, alpha=0.35)
    ax_gap.plot(sub["time_s"], sub_gdes,
                color="#f0a030", linewidth=0.7, alpha=0.30, linestyle="--")
    ax_gap.axhline(5.0, color="#e84c4c", linewidth=0.7, alpha=0.40, linestyle=":")
    max_gap = float(np.nanmax(gap_valid)) if not np.all(np.isnan(gap_valid)) else 20.0
    ax_gap.set_xlim(0, float(sub["time_s"][-1]))
    ax_gap.set_ylim(0, max_gap * 1.2)
    ax_gap.set_xlabel("Time [s]", color="#8b949e", fontsize=8)
    ax_gap.set_ylabel("Gap [m]", color="#8b949e", fontsize=8)
    ax_gap.set_title("Gap to lead", color="#e6edf3", fontsize=9)
    if cutin_t is not None:
        ax_gap.axvline(cutin_t, color="#f0a030", linewidth=1.0,
                       linestyle=":", alpha=0.7)

    # Accel panel
    max_acc = max(float(np.max(np.abs(sub["accel_mps2"]))), 0.5)
    ax_acc.plot(sub["time_s"], sub["accel_mps2"],
                color="#8b949e", linewidth=0.7, alpha=0.35)
    ax_acc.axhline(0.0, color="#8b949e", linewidth=0.6, alpha=0.5)
    ax_acc.set_xlim(0, float(sub["time_s"][-1]))
    ax_acc.set_ylim(-max_acc * 1.3, max_acc * 1.3)
    ax_acc.set_xlabel("Time [s]", color="#8b949e", fontsize=8)
    ax_acc.set_ylabel("Accel [m/s²]", color="#8b949e", fontsize=8)
    ax_acc.set_title("Acceleration", color="#e6edf3", fontsize=9)
    if cutin_t is not None:
        ax_acc.axvline(cutin_t, color="#f0a030", linewidth=1.0,
                       linestyle=":", alpha=0.7)

    # ── dynamic artists ────────────────────────────────────────────────────
    TAIL = 150

    # Ego trajectory tail
    ego_tail, = ax_map.plot([], [], color="#4c9be8", linewidth=2.0,
                             alpha=0.85, zorder=3, label="Ego")

    # Ego car patch
    ego_verts = car_corners(
        float(sub["x_m"][0]), float(sub["y_m"][0]), float(sub["yaw_rad"][0]))
    ego_patch = mpatches.Polygon(
        ego_verts, closed=True,
        facecolor="#4c9be8", edgecolor="white",
        linewidth=1.0, alpha=0.92, zorder=5)
    ax_map.add_patch(ego_patch)

    # Lead car patch (hidden until cut-in)
    lead_verts = car_corners(float(sub_lx[0]), float(sub_ly[0]),
                             float(sub_lyaw[0]))
    lead_patch = mpatches.Polygon(
        lead_verts, closed=True,
        facecolor="#e84c4c", edgecolor="white",
        linewidth=1.0, alpha=0.0, zorder=5)   # invisible initially
    ax_map.add_patch(lead_patch)

    # Lead trajectory tail
    lead_tail, = ax_map.plot([], [], color="#e84c4c", linewidth=2.0,
                              alpha=0.0, zorder=3, label="Lead vehicle")

    # Cut-in star marker (hidden initially)
    cutin_star, = ax_map.plot([], [], "*", color="#f0a030", markersize=18,
                               zorder=7, label="Cut-in point")

    # Legend
    ax_map.legend(loc="upper right", fontsize=8.5,
                  facecolor="#161b22", edgecolor="#30363d",
                  labelcolor="#e6edf3", framealpha=0.9)

    # Time / phase label
    time_text = ax_map.text(
        0.01, 0.97, "", transform=ax_map.transAxes,
        fontsize=10, color="#e6edf3", va="top",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22",
                  edgecolor="#30363d", alpha=0.85))

    # Phase label (top-right of map)
    phase_text = ax_map.text(
        0.99, 0.97, "", transform=ax_map.transAxes,
        fontsize=10, color="#e6edf3", va="top", ha="right",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22",
                  edgecolor="#30363d", alpha=0.85))

    # Speed readout
    spd_text = ax_map.text(
        0.01, 0.87, "", transform=ax_map.transAxes,
        fontsize=8.5, color="#4c9be8", va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="#161b22",
                  edgecolor="none", alpha=0.75))
    gap_text = ax_map.text(
        0.01, 0.80, "", transform=ax_map.transAxes,
        fontsize=8.5, color="#4ce87a", va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="#161b22",
                  edgecolor="none", alpha=0.75))

    # Moving dots on lower panels
    spd_dot_ego,  = ax_spd.plot([], [], "o", color="#4c9be8",  markersize=6,
                                 zorder=5, markeredgecolor="white", markeredgewidth=0.6)
    spd_dot_lead, = ax_spd.plot([], [], "o", color="#e84c4c",  markersize=6,
                                 zorder=5, markeredgecolor="white", markeredgewidth=0.6)
    gap_dot,      = ax_gap.plot([], [], "o", color="#4ce87a",  markersize=6,
                                 zorder=5, markeredgecolor="white", markeredgewidth=0.6)
    acc_dot,      = ax_acc.plot([], [], "o", color="#e8d44c",  markersize=6,
                                 zorder=5, markeredgecolor="white", markeredgewidth=0.6)

    # Time cursors
    spd_cursor, = ax_spd.plot([], [], color="white", linewidth=0.9, alpha=0.4, zorder=4)
    gap_cursor, = ax_gap.plot([], [], color="white", linewidth=0.9, alpha=0.4, zorder=4)
    acc_cursor, = ax_acc.plot([], [], color="white", linewidth=0.9, alpha=0.4, zorder=4)

    # ── phase helpers ──────────────────────────────────────────────────────
    def phase_label(frame):
        if cutin_frame is None or frame < cutin_frame:
            return "Phase 0: Free Cruise", "#3fb950"
        ve = float(sub["ego_speed_kmh"][min(frame, n_frames - 1)])
        vl = float(sub["lead_speed_kmh"][min(frame, n_frames - 1)])
        if abs(ve - vl) > 1.5:
            return "Phase 1: Hard Braking", "#e84c4c"
        return "Phase 2: Following Lead", "#58a6ff"

    # Flash counter for cut-in event (number of frames to show the warning)
    FLASH_FRAMES = max(1, int(1.5 * args.fps))

    # ── update function ────────────────────────────────────────────────────
    def update(frame):
        i = min(frame, n_frames - 1)
        t   = float(sub["time_s"][i])
        ex  = float(sub["x_m"][i])
        ey  = float(sub["y_m"][i])
        eyw = float(sub["yaw_rad"][i])
        lx  = float(sub_lx[i])
        ly  = float(sub_ly[i])
        lyw = float(sub_lyaw[i])
        ve  = float(sub["ego_speed_kmh"][i])
        vl  = float(sub["lead_speed_kmh"][i])
        gap = float(sub["gap_m"][i])
        acc = float(sub["accel_mps2"][i])
        mode = int(sub["acc_mode"][i])

        # --- Ego ---
        ego_patch.set_xy(car_corners(ex, ey, eyw))
        s0 = max(0, i - TAIL)
        ego_tail.set_data(sub["x_m"][s0:i + 1], sub["y_m"][s0:i + 1])

        # --- Lead (visible only after cut-in) ---
        lead_vis = (cutin_frame is not None and frame >= cutin_frame)
        alpha_lead = 0.92 if lead_vis else 0.0
        lead_patch.set_xy(car_corners(lx, ly, lyw))
        lead_patch.set_alpha(alpha_lead)
        lead_tail.set_alpha(0.85 if lead_vis else 0.0)
        if lead_vis:
            lt_s = max(0, i - TAIL)
            lead_tail.set_data(sub_lx[lt_s:i + 1], sub_ly[lt_s:i + 1])
        else:
            lead_tail.set_data([], [])

        # --- Cut-in flash marker ---
        if (cutin_frame is not None and
                cutin_frame <= frame < cutin_frame + FLASH_FRAMES):
            cx = float(sub["x_m"][cutin_frame])
            cy = float(sub["y_m"][cutin_frame])
            cutin_star.set_data([cx], [cy])
        else:
            cutin_star.set_data([], [])

        # --- Phase label ---
        label, color = phase_label(frame)
        phase_text.set_text(label)
        phase_text.set_color(color)
        phase_text.get_bbox_patch().set_edgecolor(color)

        # --- Text overlays ---
        pct = 100 * frame / max(n_frames - 1, 1)
        time_text.set_text(f"t = {t:.1f} s  ({pct:.0f}%)")
        spd_text.set_text(f"Ego:  {ve:.1f} km/h")
        if lead_vis:
            gap_str = f"{gap:.1f} m" if gap > 0 else "—"
            gap_text.set_text(
                f"Lead: {vl:.1f} km/h   Gap: {gap_str}")
        else:
            gap_text.set_text("")

        # --- Lower panel cursors & dots ---
        for cursor, ax in zip(
                [spd_cursor, gap_cursor, acc_cursor],
                [ax_spd,     ax_gap,     ax_acc]):
            yl = ax.get_ylim()
            cursor.set_data([t, t], [yl[0], yl[1]])

        spd_dot_ego.set_data([t], [ve])
        if lead_vis and vl > 0:
            spd_dot_lead.set_data([t], [vl])
        else:
            spd_dot_lead.set_data([], [])
        if gap > 0:
            gap_dot.set_data([t], [gap])
        else:
            gap_dot.set_data([], [])
        acc_dot.set_data([t], [acc])

        return (ego_patch, lead_patch, ego_tail, lead_tail,
                cutin_star, time_text, phase_text, spd_text, gap_text,
                spd_dot_ego, spd_dot_lead, gap_dot, acc_dot,
                spd_cursor, gap_cursor, acc_cursor)

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=max(1, 1000 // args.fps),
                         blit=False)

    # ── output ────────────────────────────────────────────────────────────
    if args.save:
        ext = os.path.splitext(args.save)[1].lower()
        print(f"Saving animation → {args.save} ...")
        if ext == ".gif":
            try:
                from PIL import Image  # noqa: F401
            except ImportError:
                sys.exit("Pillow required for GIF:  pip install pillow")
            writer = PillowWriter(fps=args.fps)
        else:
            writer = FFMpegWriter(fps=args.fps, bitrate=2500,
                                  extra_args=["-pix_fmt", "yuv420p"])
        anim.save(args.save, writer=writer, dpi=100,
                  savefig_kwargs={"facecolor": fig.get_facecolor()})
        print("Saved.")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
