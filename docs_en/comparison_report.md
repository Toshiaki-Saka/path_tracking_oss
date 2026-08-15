# Path-Tracking Algorithm Comparison Report

This report compares four path-tracking algorithms — **Pure Pursuit**,
**Stanley**, **MPC** and **MPPI** — driven around the same ~1.0 km reference
route with an identical kinematic bicycle model. All four share the same
longitudinal speed law, so the comparison isolates *steering* behaviour.

The control law behind each number below — derivation, closed-loop analysis,
parameter tables — is in [`algorithms.md`](algorithms.md).

## Test conditions

| Item | Value |
|------|-------|
| Reference route | `data/reference_route.csv` — 1871 points, 1010.0 m, bends of R = 200 / 30 / 150 / 25 m |
| Vehicle model | Kinematic bicycle, wheelbase 2.7 m |
| Control cycle | 0.01 s (100 Hz) |
| Speed profile | Curvature-limited, 1.5–8.0 m/s (lateral-accel budget 1.5 m/s²) |
| Initial speed | 30 % of the route start speed |

## Results

| Algorithm | CTE RMS [m] | CTE max [m] | Heading RMS [deg] | Steer rate RMS [deg/s] | Avg speed [km/h] | Compute [µs/cycle] | Completed |
|-----------|-------------|-------------|-------------------|------------------------|------------------|--------------------|-----------|
| Pure Pursuit | 0.038 | 0.198 | 0.235 | **1.6** | 28.0 | **0.35** | yes |
| Stanley | 0.040 | 0.141 | 0.151 | 14.3 | 28.0 | 0.37 | yes |
| MPC | **0.008** | **0.030** | **0.115** | 46.6 | 28.0 | 1.52 | yes |
| MPPI | 0.036 | 0.138 | 0.384 | 174.8 | 28.0 | 568 | yes |

All four algorithms complete the route, and all stay within 0.2 m of it — the
reference path is curvature-continuous (clothoid transitions), so none of the
controllers is ever asked to absorb a step in curvature. **MPC achieves the
lowest cross-track error**, roughly 5× tighter than the other three; **Pure
Pursuit produces the smoothest steering** at the lowest compute cost.

### Trajectory

![Trajectory comparison](fig_trajectory.png)

At map scale the four trajectories are indistinguishable from the reference
path — every algorithm tracks the route closely.

### Tracking error and steering over time

![Error time series](fig_error_timeseries.png)

The differences appear at the two tight bends, near t ≈ 55 s (R = 30 m) and
t ≈ 118 s (R = 25 m). Every algorithm's peak error occurs at the tighter of the
two:

- **Pure Pursuit** has the largest peak (0.198 m), and it occurs in the *entry
  clothoid*, not on the arc itself: the look-ahead point acts as a curvature
  preview of ℓd/3, so the vehicle turns in early (cutting inside) and
  straightens early (overshooting outside). On the constant-radius arcs its
  error is essentially zero — the look-ahead arc reproduces the path circle
  exactly ([algorithms.md §1.4–1.5](algorithms.md#14-exactness-on-constant-curvature-arcs)).
- **Stanley** holds a small but *persistent* offset through every constant-radius
  section (≈ +0.017 m on the R = 200 m sweep, +0.023 m on R = 150 m, −0.139 m on
  R = 25 m). This is geometric, not a tuning issue: Stanley drives the **front
  axle** onto the path, which leaves the rear axle inside the curve by
  L²κ/2 = R − √(R² − L²) — within 5–7 % of the measured value on all four bends
  ([algorithms.md §2.3](algorithms.md#23-steady-state-offset-on-a-curve--an-exact-result)).
  Its `cte_max` of 0.141 m *is* this steady offset, not a transient.
- **MPC** is nearly flat throughout (peak 0.030 m). The reason is the exact
  curvature feed-forward arctan(Lκ) plus a 0.52 s closed-loop time constant,
  applied to the same reference point the metric measures — note that this
  implementation does not preview *future* curvature; the horizon only shapes
  the feedback gains
  ([algorithms.md §3.1, §3.5](algorithms.md#31-error-state-and-prediction-model)).
  Its high steering rate is a period-9 sawtooth caused by nearest-point
  quantisation of the 0.54 m sampled path, not by the LQR.
- **MPPI** tracks well on average but its trace is visibly noisy (±0.05 m on the
  straights) — the direct signature of its stochastic sampling.

### Aggregate metrics

![Aggregate metrics](fig_metrics.png)

## Discussion

| Algorithm | Strengths observed | Trade-offs observed |
|-----------|--------------------|--------------------|
| **Pure Pursuit** | Simplest; smoothest steering by a wide margin (steer-rate RMS 1.6 deg/s); lowest compute cost. | Largest peak error at the tight bend — corner-cutting on entry, overshoot on exit; no direct lateral-error feedback. |
| **Stanley** | Low compute cost; speed-independent convergence time constant (1/k = 0.4 s); corrects lateral + heading error directly. | Steady-state offset L²κ/2 on constant-curvature sections, because it regulates the front axle; steering busier than Pure Pursuit. |
| **MPC** | Best tracking accuracy by ~5×; exact curvature feed-forward on the measured reference point; jointly weighs error and control effort. | Higher steering activity than the geometric trackers (amplified map quantisation); needs a model and a Riccati solve (Eigen dependency). |
| **MPPI** | Handles non-convex / non-differentiable costs; accuracy comparable to the geometric trackers; naturally parallel. | By far the highest compute cost; noisiest steering; result depends on sample count and temperature. |

### Summary

- On a smooth, curvature-continuous route with a kinematic model, all four
  methods are accurate in absolute terms (< 0.2 m peak). The interesting spread
  is not "does it track" but *how* each one fails: Pure Pursuit in the clothoid
  transitions, Stanley as a geometric offset on arcs, MPPI as sampling noise.
- **MPC** wins on accuracy for two structural reasons: it feeds forward the
  exact steady-state steering arctan(Lκ), and it regulates the very point the
  metric is measured at — the two error sources that cost the geometric
  trackers their accuracy here.
- **MPPI**'s generality (non-convex, non-differentiable costs) is not exercised
  by a single-route kinematic benchmark, so here it only pays the cost: ~1600×
  the per-cycle time of the geometric trackers in this single-threaded
  reference build.

## Reproducing

```bash
python tools/generate_reference_route.py     # optional; the CSV is committed
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cd build && ./path_tracking_compare
python ../tools/plot_results.py . ../data/reference_route.csv
```

Numbers may vary slightly with compiler, optimization flags and CPU; MPPI
uses a fixed RNG seed so its trajectory is reproducible.
