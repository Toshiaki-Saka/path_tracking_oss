# SPDX-License-Identifier: Apache-2.0
"""
generate_reference_route.py - Build the synthetic reference route CSV.

The reference path used by this project is generated procedurally from a
curvature-vs-arc-length profile: straights joined to constant-radius arcs by
linear-curvature (clothoid) transitions, which is how real road geometry is
laid out. No third-party map data is involved, so the resulting CSV carries no
licensing obligations beyond this repository's own.

Usage:
    python tools/generate_reference_route.py [output_csv]

    output_csv : destination (default: data/reference_route.csv)

Output columns: x_m, y_m, yaw, curvature
"""

import math
import os
import sys

# ---------------------------------------------------------------------------
# Route definition
# ---------------------------------------------------------------------------
# Control points of the curvature profile: (arc length [m], curvature [1/m]).
# Curvature is linearly interpolated between consecutive points, so a pair with
# equal curvature is a straight (kappa = 0) or a constant-radius arc, and a pair
# with differing curvature is a clothoid transition. Positive curvature turns
# left. The profile below is ~1.0 km with four bends (R = 200, 30, 150 and
# 25 m), giving the tracking controllers a mix of gentle sweeps, two tight
# junction-style turns, and long straights to settle on.
PROFILE = [
    (0.0,     0.0),          # start of a 130 m straight
    (130.0,   0.0),
    (150.0,   1.0 / 200.0),  # clothoid into a gentle left sweep
    (240.0,   1.0 / 200.0),  # R = 200 m arc, 90 m long
    (260.0,   0.0),          # clothoid out
    (400.0,   0.0),          # 140 m straight
    (420.0,  -1.0 / 30.0),   # clothoid into a junction-style right turn
    (447.0,  -1.0 / 30.0),   # R = 30 m arc, 27 m long
    (467.0,   0.0),          # clothoid out
    (620.0,   0.0),          # 153 m straight
    (640.0,   1.0 / 150.0),  # clothoid into a long left sweep
    (760.0,   1.0 / 150.0),  # R = 150 m arc, 120 m long
    (780.0,   0.0),          # clothoid out
    (900.0,   0.0),          # 120 m straight
    (915.0,  -1.0 / 25.0),   # clothoid into the tightest bend
    (945.0,  -1.0 / 25.0),   # R = 25 m arc, 30 m long
    (960.0,   0.0),          # clothoid out
    (1010.0,  0.0),          # 50 m run-out straight
]

INITIAL_YAW = 0.0    # heading at s = 0 [rad]
POINT_SPACING = 0.54  # output sample spacing along the path [m]
INTEGRATION_STEP = 0.01  # internal integration step [m]


def curvature_at(s: float) -> float:
    """Curvature at arc length `s`, linearly interpolated over PROFILE."""
    if s <= PROFILE[0][0]:
        return PROFILE[0][1]
    for (s0, k0), (s1, k1) in zip(PROFILE, PROFILE[1:]):
        if s <= s1:
            t = (s - s0) / (s1 - s0)
            return k0 + t * (k1 - k0)
    return PROFILE[-1][1]


def wrap_pi(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def build_route():
    """Integrate the curvature profile into (x, y, yaw, curvature) samples."""
    total_s = PROFILE[-1][0]
    n_out = int(round(total_s / POINT_SPACING)) + 1
    ds_out = total_s / (n_out - 1)
    substeps = max(1, int(round(ds_out / INTEGRATION_STEP)))
    h = ds_out / substeps

    x, y, yaw = 0.0, 0.0, INITIAL_YAW
    rows = [(x, y, wrap_pi(yaw), curvature_at(0.0))]

    s = 0.0
    for i in range(1, n_out):
        for _ in range(substeps):
            # Midpoint rule: advance the heading half a step, move, then finish.
            k_mid = curvature_at(s + 0.5 * h)
            yaw_mid = yaw + 0.5 * h * k_mid
            x += h * math.cos(yaw_mid)
            y += h * math.sin(yaw_mid)
            yaw += h * k_mid
            s += h
        rows.append((x, y, wrap_pi(yaw), curvature_at(min(s, total_s))))

    return rows


def main() -> int:
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join("data", "reference_route.csv")
    rows = build_route()

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("x_m,y_m,yaw,curvature\n")
        for x, y, yaw, k in rows:
            f.write(f"{x:.6f},{y:.6f},{yaw:.9f},{k:.9f}\n")

    length = sum(
        math.hypot(rows[i][0] - rows[i - 1][0], rows[i][1] - rows[i - 1][1])
        for i in range(1, len(rows))
    )
    k_max = max(abs(r[3]) for r in rows)
    print(f"wrote {out_path}: {len(rows)} points, {length:.1f} m, "
          f"|kappa|max {k_max:.4f} (R_min {1.0 / k_max:.1f} m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
