#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
animate_results.py  --  Animate path-tracking simulation results.

All four algorithm vehicles are shown simultaneously on the XY map,
synced by simulation step.  A lower panel shows cross-track error over
time with a live cursor.

Usage
-----
  python tools/animate_results.py [result_dir] [route_csv] [options]

  result_dir : directory with trace_*.csv files  (default: build/Release)
  route_csv  : reference path CSV                (default: data/reference_route.csv)

Options
  --step N    use every N-th simulation step  (default: 20)
  --fps  N    playback frame rate             (default: 30)
  --save-frames DIR  write animation_frame0.png / animation_frame_mid.png to DIR
  --save FILE export to .gif (needs Pillow) or .mp4 (needs FFmpeg)
              omit to show an interactive window
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
# Constants
# ---------------------------------------------------------------------------
ALGORITHMS = ["PurePursuit", "Stanley", "MPC", "MPPI"]
COLORS = {
    "PurePursuit": "#4c9be8",
    "Stanley":     "#e84c4c",
    "MPC":         "#4ce87a",
    "MPPI":        "#c47ce8",
}
CAR_LEN = 8.0   # display metres (exaggerated for visibility on 1 km map)
CAR_WID = 3.5


# ---------------------------------------------------------------------------
# Data loaders
# ---------------------------------------------------------------------------

def load_route(path: str):
    xs, ys = [], []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return np.asarray(xs), np.asarray(ys)


def load_trace(path: str) -> dict:
    cols = ("step", "time_s", "x_m", "y_m", "yaw_rad",
            "speed_kmh", "steer_deg", "cte_m", "heading_err_deg")
    data = {k: [] for k in cols}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            for k in cols:
                data[k].append(float(row[k]))
    return {k: np.asarray(v) for k, v in data.items()}


# ---------------------------------------------------------------------------
# Geometry helper
# ---------------------------------------------------------------------------

def car_corners(x: float, y: float, yaw: float,
                length: float = CAR_LEN, width: float = CAR_WID) -> np.ndarray:
    """Return 4x2 array of car rectangle corners in world frame."""
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
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("result_dir", nargs="?", default="build/Release")
    ap.add_argument("route_csv",  nargs="?",
                    default="data/reference_route.csv")
    ap.add_argument("--step", type=int, default=20,
                    help="subsample every N-th step (default 20)")
    ap.add_argument("--fps",  type=int, default=30,
                    help="animation frame rate (default 30)")
    ap.add_argument("--save", default="",
                    help="save to FILE (.gif or .mp4) instead of interactive")
    ap.add_argument("--save-frames", default="", metavar="DIR",
                    help="write the documentation still frames to DIR and exit")
    args = ap.parse_args()

    # Choose backend before importing pyplot
    if not args.save and not args.save_frames:
        try:
            matplotlib.use("TkAgg")
        except Exception:
            pass  # fall back to whatever is available

    # ── load data ──────────────────────────────────────────────────────────
    if not os.path.exists(args.route_csv):
        sys.exit(f"Route CSV not found: {args.route_csv}")

    rx, ry = load_route(args.route_csv)

    traces: dict = {}
    for algo in ALGORITHMS:
        p = os.path.join(args.result_dir, f"trace_{algo}.csv")
        if os.path.exists(p):
            traces[algo] = load_trace(p)
            print(f"  loaded {p}  ({len(traces[algo]['step'])} steps)")
        else:
            print(f"  skip: {p}")

    if not traces:
        sys.exit("No trace CSVs found in " + args.result_dir)

    algos = [a for a in ALGORITHMS if a in traces]

    # subsample every N-th step
    sub = {a: {k: v[:: args.step] for k, v in traces[a].items()} for a in algos}
    n_frames = max(len(sub[a]["time_s"]) for a in algos)
    print(f"\n  route     : {args.route_csv}")
    print(f"  algorithms: {algos}")
    print(f"  frames    : {n_frames}  (step={args.step}, fps={args.fps})")
    print(f"  duration  : ~{n_frames / args.fps:.1f} s")

    # ── figure ─────────────────────────────────────────────────────────────
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor("#0d1117")

    gs      = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.35,
                               left=0.07, right=0.97, top=0.93, bottom=0.07)
    ax_map  = fig.add_subplot(gs[:2, :])   # top 2/3 full-width: XY map
    ax_cte  = fig.add_subplot(gs[2,  0])   # bottom-left: CTE
    ax_spd  = fig.add_subplot(gs[2,  1])   # bottom-right: speed

    MAP_BG  = "#161b22"
    PANEL_BG = "#0d1117"
    for ax in (ax_map, ax_cte, ax_spd):
        ax.set_facecolor(MAP_BG)
        ax.grid(True, alpha=0.12, color="#8b949e", linewidth=0.5)
        for spine in ax.spines.values():
            spine.set_edgecolor("#30363d")

    # reference route
    ax_map.plot(rx, ry, "--", color="#8b949e", linewidth=1.2,
                alpha=0.6, label="Reference", zorder=1)
    ax_map.plot(rx[0],  ry[0],  "o", color="#3fb950", markersize=9,
                zorder=6, label="Start", markeredgecolor="white", markeredgewidth=0.8)
    ax_map.plot(rx[-1], ry[-1], "s", color="#f78166", markersize=9,
                zorder=6, label="Goal",  markeredgecolor="white", markeredgewidth=0.8)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("x [m]", color="#8b949e")
    ax_map.set_ylabel("y [m]", color="#8b949e")
    ax_map.set_title("Path-tracking animation", color="#e6edf3", fontsize=13,
                     fontweight="bold", pad=10)
    ax_map.tick_params(colors="#8b949e")

    # CTE axes setup
    max_cte = max(float(np.max(np.abs(sub[a]["cte_m"]))) for a in algos)
    ax_cte.set_ylim(-max_cte * 1.25, max_cte * 1.25)
    ax_cte.axhline(0, color="#8b949e", linewidth=0.8, alpha=0.6)
    ax_cte.set_xlabel("Time [s]", color="#8b949e")
    ax_cte.set_ylabel("CTE [m]",  color="#8b949e")
    ax_cte.set_title("Cross-track error", color="#e6edf3", fontsize=10)
    ax_cte.tick_params(colors="#8b949e")

    max_t = max(float(sub[a]["time_s"][-1]) for a in algos)
    ax_cte.set_xlim(0, max_t)

    # speed axes setup
    max_spd = max(float(np.max(sub[a]["speed_kmh"])) for a in algos)
    ax_spd.set_ylim(0, max_spd * 1.25)
    ax_spd.set_xlabel("Time [s]", color="#8b949e")
    ax_spd.set_ylabel("Speed [km/h]", color="#8b949e")
    ax_spd.set_title("Speed profile", color="#e6edf3", fontsize=10)
    ax_spd.tick_params(colors="#8b949e")
    ax_spd.set_xlim(0, max_t)

    # ── per-algorithm artists ──────────────────────────────────────────────
    TAIL = 200   # frames kept in the tail

    tail_lines  = {}
    car_patches = {}
    cte_full    = {}
    cte_dots    = {}
    spd_full    = {}
    spd_dots    = {}

    for algo in algos:
        col = COLORS[algo]
        t   = sub[algo]

        # trajectory tail
        tail_lines[algo], = ax_map.plot(
            [], [], color=col, linewidth=2.0, alpha=0.85, zorder=3, label=algo)

        # car body
        verts = car_corners(t["x_m"][0], t["y_m"][0], t["yaw_rad"][0])
        patch = mpatches.Polygon(verts, closed=True,
                                 facecolor=col, edgecolor="white",
                                 linewidth=0.9, alpha=0.92, zorder=5)
        ax_map.add_patch(patch)
        car_patches[algo] = patch

        # full CTE curve (static)
        cte_full[algo], = ax_cte.plot(
            t["time_s"], t["cte_m"],
            color=col, linewidth=0.9, alpha=0.45, label=algo)

        # moving dot on CTE
        cte_dots[algo], = ax_cte.plot(
            [], [], "o", color=col, markersize=5, zorder=5,
            markeredgecolor="white", markeredgewidth=0.5)

        # full speed curve (static)
        spd_full[algo], = ax_spd.plot(
            t["time_s"], t["speed_kmh"],
            color=col, linewidth=0.9, alpha=0.45, label=algo)

        # moving dot on speed
        spd_dots[algo], = ax_spd.plot(
            [], [], "o", color=col, markersize=5, zorder=5,
            markeredgecolor="white", markeredgewidth=0.5)

    # vertical time cursors on lower plots
    cte_cursor, = ax_cte.plot([], [], color="white", linewidth=0.9,
                               alpha=0.5, zorder=4)
    spd_cursor, = ax_spd.plot([], [], color="white", linewidth=0.9,
                               alpha=0.5, zorder=4)

    # legends
    ax_map.legend(loc="upper right", fontsize=8.5,
                  facecolor="#161b22", edgecolor="#30363d",
                  labelcolor="#e6edf3", framealpha=0.9)
    ax_cte.legend(loc="upper right", fontsize=7.5,
                  facecolor="#161b22", edgecolor="#30363d",
                  labelcolor="#e6edf3", framealpha=0.9)
    ax_spd.legend(loc="upper right", fontsize=7.5,
                  facecolor="#161b22", edgecolor="#30363d",
                  labelcolor="#e6edf3", framealpha=0.9)

    # clock + progress text
    time_text = ax_map.text(
        0.01, 0.97, "", transform=ax_map.transAxes,
        fontsize=10, color="#e6edf3", va="top",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22",
                  edgecolor="#30363d", alpha=0.85))

    # per-algo speed readout (top-left corner labels)
    speed_texts = {}
    for i, algo in enumerate(algos):
        speed_texts[algo] = ax_map.text(
            0.01, 0.89 - i * 0.07, "",
            transform=ax_map.transAxes,
            fontsize=8.5, color=COLORS[algo], va="top",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="#161b22",
                      edgecolor="none", alpha=0.75))

    # ── animation update ───────────────────────────────────────────────────
    def update(frame: int):
        cur_t = None
        for algo in algos:
            t   = sub[algo]
            idx = min(frame, len(t["time_s"]) - 1)
            if cur_t is None:
                cur_t = float(t["time_s"][idx])

            # car rectangle
            verts = car_corners(
                float(t["x_m"][idx]),
                float(t["y_m"][idx]),
                float(t["yaw_rad"][idx]))
            car_patches[algo].set_xy(verts)

            # tail
            s = max(0, idx - TAIL)
            tail_lines[algo].set_data(t["x_m"][s:idx + 1],
                                      t["y_m"][s:idx + 1])

            # CTE dot
            cte_dots[algo].set_data([t["time_s"][idx]], [t["cte_m"][idx]])

            # speed dot
            spd_dots[algo].set_data([t["time_s"][idx]], [t["speed_kmh"][idx]])

            # speed readout
            speed_texts[algo].set_text(
                f"{algo}: {t['speed_kmh'][idx]:.1f} km/h  "
                f"CTE={t['cte_m'][idx]:+.3f} m")

        # time cursor
        if cur_t is not None:
            cte_cursor.set_data([cur_t, cur_t], [ax_cte.get_ylim()[0],
                                                  ax_cte.get_ylim()[1]])
            spd_cursor.set_data([cur_t, cur_t], [ax_spd.get_ylim()[0],
                                                  ax_spd.get_ylim()[1]])
            pct = 100 * frame / max(n_frames - 1, 1)
            time_text.set_text(f"t = {cur_t:.1f} s   ({pct:.0f}%)")

        artists = (list(tail_lines.values()) +
                   list(car_patches.values()) +
                   list(cte_dots.values()) +
                   list(spd_dots.values()) +
                   list(speed_texts.values()) +
                   [cte_cursor, spd_cursor, time_text])
        return artists

    if args.save_frames:
        # Still frames for the documentation: first frame and mid-run.
        os.makedirs(args.save_frames, exist_ok=True)
        for name, frame in (("animation_frame0", 0),
                            ("animation_frame_mid", (n_frames - 1) // 2)):
            update(frame)
            out = os.path.join(args.save_frames, name + ".png")
            fig.savefig(out, dpi=110, facecolor=fig.get_facecolor())
            print(f"  wrote {out}")
        return 0

    anim = FuncAnimation(
        fig, update,
        frames=n_frames,
        interval=max(1, 1000 // args.fps),
        blit=False)   # blit=False for patch compatibility

    # ── output ─────────────────────────────────────────────────────────────
    if args.save:
        ext = os.path.splitext(args.save)[1].lower()
        print(f"Saving to {args.save} ...")
        if ext == ".gif":
            try:
                from PIL import Image  # noqa: F401  (Pillow check)
            except ImportError:
                sys.exit("Pillow is required for GIF export:  pip install pillow")
            writer = PillowWriter(fps=args.fps)
        else:
            writer = FFMpegWriter(fps=args.fps, bitrate=2500,
                                  extra_args=["-pix_fmt", "yuv420p"])
        anim.save(args.save, writer=writer, dpi=110,
                  savefig_kwargs={"facecolor": fig.get_facecolor()})
        print("Saved.")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
