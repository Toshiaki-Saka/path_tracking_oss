#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
plot_results.py - Visualise path-tracking comparison results.

Reads the trace CSVs (trace_<Algorithm>.csv) and the summary CSV
(comparison_summary.csv) produced by `path_tracking_compare`, then writes
PNG figures comparing the four algorithms.

Usage:
    python tools/plot_results.py [result_dir] [route_csv]

    result_dir : directory containing trace_*.csv and comparison_summary.csv
                 (default: build)
    route_csv  : reference path CSV (default: data/reference_route.csv)
"""
import os
import sys
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ALGORITHMS = ["PurePursuit", "Stanley", "MPC", "MPPI"]
COLORS = {
    "PurePursuit": "#1f77b4",
    "Stanley":     "#d62728",
    "MPC":         "#2ca02c",
    "MPPI":        "#9467bd",
}


def read_csv_dict(path):
    """Read a CSV file into a list of dict rows (all values as float where possible)."""
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


def load_route(route_csv):
    xs, ys = [], []
    with open(route_csv, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return xs, ys


def col(rows, name):
    return [r[name] for r in rows]


def plot_trajectories(result_dir, route_csv, traces, out_path):
    """XY trajectory of every algorithm overlaid on the reference path."""
    fig, ax = plt.subplots(figsize=(10, 8))

    rx, ry = load_route(route_csv)
    ax.plot(rx, ry, "--", color="0.5", linewidth=1.4, label="Reference path")

    for algo in ALGORITHMS:
        if algo not in traces:
            continue
        t = traces[algo]
        ax.plot(col(t, "x_m"), col(t, "y_m"),
                color=COLORS[algo], linewidth=1.6, label=algo)

    ax.plot(rx[0], ry[0], "ko", markersize=8)
    ax.annotate("start", (rx[0], ry[0]), textcoords="offset points",
                xytext=(8, 8))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Path-tracking trajectory comparison")
    ax.legend(loc="best")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_path}")


def plot_error_timeseries(traces, out_path):
    """Cross-track error and steering angle vs. time."""
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    for algo in ALGORITHMS:
        if algo not in traces:
            continue
        t = traces[algo]
        axes[0].plot(col(t, "time_s"), col(t, "cte_m"),
                     color=COLORS[algo], linewidth=1.0, label=algo)
        axes[1].plot(col(t, "time_s"), col(t, "steer_deg"),
                     color=COLORS[algo], linewidth=1.0, label=algo)

    axes[0].axhline(0.0, color="0.6", linewidth=0.8)
    axes[0].set_ylabel("Cross-track error [m]")
    axes[0].set_title("Tracking error over time")
    axes[0].legend(loc="upper right", ncol=4)
    axes[0].grid(True, alpha=0.3)

    axes[1].axhline(0.0, color="0.6", linewidth=0.8)
    axes[1].set_ylabel("Steering angle [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Steering command over time")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_path}")


def plot_metrics_bars(summary_rows, out_path):
    """Bar charts of the aggregate metrics."""
    algos = [r["algorithm"] for r in summary_rows]
    colors = [COLORS.get(a, "0.5") for a in algos]

    metrics = [
        ("cte_rms_m",            "CTE RMS [m]",            "lower is better"),
        ("cte_max_m",            "CTE max [m]",            "lower is better"),
        ("steer_rate_rms_dps",   "Steering rate RMS [deg/s]", "lower = smoother"),
        ("compute_us_per_cycle", "Compute time [us/cycle]", "log scale"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for ax, (key, title, note) in zip(axes.flat, metrics):
        vals = [r[key] for r in summary_rows]
        bars = ax.bar(algos, vals, color=colors)
        ax.set_title(f"{title}\n({note})", fontsize=11)
        ax.grid(True, axis="y", alpha=0.3)
        if key == "compute_us_per_cycle":
            ax.set_yscale("log")
        for b, v in zip(bars, vals):
            ax.annotate(f"{v:.3g}", (b.get_x() + b.get_width() / 2,
                                     b.get_height()),
                        textcoords="offset points", xytext=(0, 3),
                        ha="center", fontsize=9)

    fig.suptitle("Aggregate performance metrics", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_path}")


def main():
    result_dir = sys.argv[1] if len(sys.argv) > 1 else "build"
    route_csv = sys.argv[2] if len(sys.argv) > 2 else "data/reference_route.csv"

    traces = {}
    for algo in ALGORITHMS:
        p = os.path.join(result_dir, f"trace_{algo}.csv")
        if os.path.exists(p):
            traces[algo] = read_csv_dict(p)
        else:
            print(f"  warning: missing {p}")

    if not traces:
        print("Error: no trace CSVs found in", result_dir)
        return 1

    summary_path = os.path.join(result_dir, "comparison_summary.csv")
    summary_rows = read_csv_dict(summary_path) if os.path.exists(summary_path) else []

    print("Generating figures ...")
    plot_trajectories(result_dir, route_csv, traces,
                      os.path.join(result_dir, "fig_trajectory.png"))
    plot_error_timeseries(traces,
                          os.path.join(result_dir, "fig_error_timeseries.png"))
    if summary_rows:
        plot_metrics_bars(summary_rows,
                          os.path.join(result_dir, "fig_metrics.png"))
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
