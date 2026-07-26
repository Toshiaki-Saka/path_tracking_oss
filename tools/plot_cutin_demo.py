#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
plot_cutin_demo.py - Visualize the cut-in vehicle following control demo.

Reads cutin_trace.csv produced by cutin_demo and generates a 4-panel figure:
  (1) XY trajectory colored by ego speed, with cut-in marker
  (2) Speed over time: ego vs lead vehicle
  (3) Gap to lead vehicle vs time, with desired-gap reference
  (4) Longitudinal acceleration vs time

Usage:
    python tools/plot_cutin_demo.py [result_dir] [route_csv]

    result_dir : directory containing cutin_trace.csv
                 (default: build/Release)
    route_csv  : reference path CSV (default: data/reference_route.csv)
"""

import os
import sys
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection


def read_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            conv = {}
            for k, v in row.items():
                try:
                    conv[k] = float(v)
                except (ValueError, TypeError):
                    conv[k] = v
            rows.append(conv)
    return rows


def col(rows, name):
    return [r[name] for r in rows]


def load_route(csv_path):
    xs, ys = [], []
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return xs, ys


def main():
    result_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join("build", "Release")
    route_csv  = sys.argv[2] if len(sys.argv) > 2 else os.path.join("data", "reference_route.csv")

    trace_path = os.path.join(result_dir, "cutin_trace.csv")
    if not os.path.exists(trace_path):
        print(f"Error: {trace_path} not found. Run cutin_demo first.")
        return 1

    rows = read_csv(trace_path)
    if not rows:
        print("Error: cutin_trace.csv is empty.")
        return 1

    t       = np.array(col(rows, "time_s"))
    x       = np.array(col(rows, "x_m"))
    y       = np.array(col(rows, "y_m"))
    v_ego   = np.array(col(rows, "ego_speed_kmh"))
    v_lead  = np.array(col(rows, "lead_speed_kmh"))
    gap     = np.array(col(rows, "gap_m"))
    accel   = np.array(col(rows, "accel_mps2"))
    mode    = np.array(col(rows, "acc_mode"))

    # Cut-in moment: first step where acc_mode == 1
    cutin_indices = np.where(mode == 1.0)[0]
    cutin_idx = int(cutin_indices[0]) if len(cutin_indices) else None
    cutin_t   = float(t[cutin_idx]) if cutin_idx is not None else None

    # Desired gap: max(min_gap=5m, time_gap=2s * v_ego_mps)
    v_ego_mps = v_ego / 3.6
    gap_des = np.maximum(5.0, 2.0 * v_ego_mps)

    # Mask gap where no lead is present (gap == -1)
    gap_valid = np.where(gap > 0, gap, np.nan)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        "Cut-in Following Control Demo\n"
        "Lead vehicle cuts in at 15 m ahead at 10.8 km/h; "
        "ego brakes from ~29 km/h and follows",
        fontsize=13, fontweight="bold"
    )

    # -------------------------------------------------------------------------
    # (1) XY Trajectory — colored by ego speed
    # -------------------------------------------------------------------------
    ax = axes[0, 0]

    if os.path.exists(route_csv):
        rx, ry = load_route(route_csv)
        ax.plot(rx, ry, "--", color="0.82", linewidth=1.2,
                label="Reference path", zorder=1)

    # Colored line by speed
    pts  = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc   = LineCollection(segs, cmap="RdYlGn", linewidth=2.2, zorder=2)
    lc.set_array(v_ego[:-1])
    lc.set_clim(0, float(np.nanmax(v_ego)) * 1.05)
    ax.add_collection(lc)
    cbar = plt.colorbar(lc, ax=ax)
    cbar.set_label("Ego speed [km/h]", fontsize=9)

    if cutin_idx is not None:
        ax.plot(x[cutin_idx], y[cutin_idx], "*",
                color="red", markersize=15, zorder=5,
                label=f"Cut-in (t={cutin_t:.1f} s)")

    ax.plot(x[0],  y[0],  "o", color="#2ca02c", markersize=9, label="Start", zorder=4)
    ax.plot(x[-1], y[-1], "s", color="#1f77b4", markersize=9, label="End",   zorder=4)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory (colored by speed)")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)

    # -------------------------------------------------------------------------
    # (2) Speed vs Time
    # -------------------------------------------------------------------------
    ax = axes[0, 1]
    ax.plot(t, v_ego,  color="#1f77b4", linewidth=2.0, label="Ego speed")
    lead_mask = v_lead > 0
    ax.plot(t[lead_mask], v_lead[lead_mask],
            color="#d62728", linewidth=1.6, linestyle="--", label="Lead speed")

    if cutin_t is not None:
        ax.axvline(cutin_t, color="orange", linewidth=1.8,
                   linestyle=":", label=f"Cut-in (t={cutin_t:.1f} s)")

    ax.fill_between(t, v_ego, v_lead,
                    where=(mode == 1) & (v_ego > v_lead),
                    alpha=0.15, color="#d62728", label="Speed excess (to shed)")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [km/h]")
    ax.set_title("Speed over time")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # Phase labels
    if cutin_t is not None:
        mid_free   = cutin_t * 0.5
        mid_follow = cutin_t + (t[-1] - cutin_t) * 0.4
        ymax = float(np.nanmax(v_ego)) * 1.05
        ax.text(mid_free,   ymax * 0.92, "Phase 0\nFree cruise",
                ha="center", va="top", fontsize=8, color="0.4")
        ax.text(mid_follow, ymax * 0.92, "Phase 1–2\nBrake → Follow",
                ha="center", va="top", fontsize=8, color="#d62728")

    # -------------------------------------------------------------------------
    # (3) Gap vs Time
    # -------------------------------------------------------------------------
    ax = axes[1, 0]
    ax.plot(t, gap_valid, color="#2ca02c", linewidth=2.0, label="Actual gap")
    ax.plot(t, gap_des,   color="orange",  linewidth=1.5, linestyle="--",
            label="Desired gap (2 s headway)")
    ax.axhline(5.0, color="red", linewidth=1.2, linestyle=":",
               alpha=0.8, label="Min gap (5 m)")

    if cutin_t is not None:
        ax.axvline(cutin_t, color="orange", linewidth=1.8, linestyle=":")
        ax.annotate("Cut-in",
                    xy=(cutin_t, float(np.nanmax(gap_valid[np.isfinite(gap_valid)])) * 0.85),
                    fontsize=8, color="orange",
                    xytext=(cutin_t + (t[-1]-cutin_t)*0.05,
                            float(np.nanmax(gap_valid[np.isfinite(gap_valid)])) * 0.90),
                    arrowprops=dict(arrowstyle="->", color="orange"))

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Gap to lead vehicle [m]")
    ax.set_title("Gap to lead vehicle")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)

    # -------------------------------------------------------------------------
    # (4) Longitudinal acceleration vs Time
    # -------------------------------------------------------------------------
    ax = axes[1, 1]
    ax.fill_between(t, 0, accel,
                    where=accel >= 0, color="#2ca02c", alpha=0.5, label="Accelerate")
    ax.fill_between(t, 0, accel,
                    where=accel <  0, color="#d62728", alpha=0.5, label="Brake")
    ax.plot(t, accel, color="0.2", linewidth=1.0)
    ax.axhline(0.0, color="0.5", linewidth=0.8)

    if cutin_t is not None:
        ax.axvline(cutin_t, color="orange", linewidth=1.8,
                   linestyle=":", label=f"Cut-in (t={cutin_t:.1f} s)")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Longitudinal acceleration [m/s²]")
    ax.set_title("Acceleration command")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)

    # -------------------------------------------------------------------------
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out_path = os.path.join(result_dir, "fig_cutin_demo.png")
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
