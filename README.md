# path_tracking_oss

**English** | [日本語](README.ja.md)

An open-source C++17 sample for **vehicle path tracking** that implements and
**compares four classic path-following algorithms** on the same reference
route and the same kinematic vehicle model.

| Algorithm | Family | Idea |
|-----------|--------|------|
| **Pure Pursuit** | Geometric | Chase a look-ahead point on the path. |
| **Stanley** | Geometric (front-axle) | Correct heading error + cross-track error at the front axle. |
| **MPC** | Optimization | Linear time-varying MPC on the lateral-error dynamics, solved by the backward Riccati recursion. |
| **MPPI** | Sampling | Model Predictive Path Integral: roll out many noisy steering sequences and take a cost-weighted average. |

The progression Pure Pursuit → Stanley → MPC → MPPI mirrors the historical
move from *geometric heuristics* to *model-predictive optimization* to
*sampling-based optimization*.

## Repository layout

```
path_tracking_oss/
├── CMakeLists.txt
├── README.md
├── include/path_tracking/      # header-only library
│   ├── types.hpp               #   geometric types, Path, ControlCommand
│   ├── vehicle.hpp             #   kinematic bicycle model
│   ├── path_io.hpp             #   CSV loading, speed profile, nearest search
│   ├── controller.hpp          #   abstract Controller interface
│   ├── pure_pursuit.hpp        #   Pure Pursuit controller
│   ├── stanley.hpp             #   Stanley controller
│   ├── mpc.hpp                 #   Model Predictive Control (Eigen)
│   ├── mppi.hpp                #   Model Predictive Path Integral
│   ├── acc_controller.hpp      #   ACC / gap-following (cut-in demo)
│   └── simulator.hpp           #   closed-loop simulation + metrics
├── src/
│   ├── main.cpp                # comparison benchmark
│   └── cutin_demo.cpp          # ACC cut-in following demo
├── tests/path_tracking_tests.cpp   # regression tests (ctest)
├── tools/                      # route generator, matplotlib figure / animation scripts
├── data/reference_route.csv     # reference route (~1.0 km, synthetic)
├── docs_en/                    # generated report / figures (English)
└── docs_ja/                    # generated report / figures (Japanese)
```

## Dependencies

- A C++17 compiler (GCC, Clang or MSVC).
- CMake ≥ 3.16.
- **Eigen3** (≥ 3.3) — used by the MPC controller for matrix algebra.
  - Ubuntu/Debian: `sudo apt install libeigen3-dev`
  - macOS: `brew install eigen`
  - vcpkg: `vcpkg install eigen3`, then pass
    `-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake`
- Python 3 with `matplotlib` (only for the optional figure script).

Pure Pursuit, Stanley and MPPI depend solely on the C++ standard library;
only MPC links Eigen.

## Quick start (Windows / PowerShell)

A single helper script — **`run_simulation.ps1`** — configures CMake, builds,
runs the simulations, and launches the animations in one step. It replaces the
older `build_and_run.ps1` / `build_cutin_demo.ps1` scripts; pick which demo to
run with `-Target`:

```powershell
# Build and run both demos (default)
.\run_simulation.ps1

# Path-tracking comparison benchmark only
.\run_simulation.ps1 -Target compare

# ACC cut-in following demo only (4-panel plot + animation)
.\run_simulation.ps1 -Target cutin
```

Options:

| Flag | Meaning |
|------|---------|
| `-Target compare\|cutin\|all` | which demo to build/run (default `all`) |
| `-BuildType Release\|Debug` | build configuration (default `Release`) |
| `-Reconfigure` | force a fresh CMake configure |
| `-NoAnimate` | build + run only, skip the matplotlib animation/plot |
| `-Save FILE` | save the animation to `FILE` (MP4/GIF) instead of opening a window |

```powershell
# Build in Debug and skip the animation
.\run_simulation.ps1 -BuildType Debug -NoAnimate

# Save the animation to a file instead of showing a window
.\run_simulation.ps1 -Target compare -Save demo.mp4
```

For the ACC cut-in demo details see
[ACC cut-in following demo](#acc-cut-in-following-demo).

For a manual, cross-platform build see below.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

```bash
cd build
./path_tracking_compare [route.csv] [output_dir]
```

- `route.csv` — reference path (default `data/reference_route.csv`).
- `output_dir` — where trace CSVs are written (default: current directory).

The program runs all four controllers, writes one trace CSV per algorithm
(`trace_<Algorithm>.csv`) plus `comparison_summary.csv`, and prints a
side-by-side performance table.

### Reference-path CSV format

A header row followed by one point per line:

```
x_m, y_m, lat, lon, yaw, curvature
```

Only `x_m`, `y_m`, `yaw` and `curvature` are used. Arc length and a
curvature-limited reference speed profile are derived automatically.

## Figures

```bash
python tools/plot_results.py build data/reference_route.csv
```

Produces, in the result directory:

- `fig_trajectory.png` — XY trajectories overlaid on the reference path.
- `fig_error_timeseries.png` — cross-track error and steering vs. time.
- `fig_metrics.png` — bar charts of the aggregate metrics.

An animated comparison of all four controllers on the reference route
(`tools/animate_results.py`):

![Path-tracking comparison animation](docs_en/animation_demo.gif)

## Results

Four controllers on the same ~1.0 km route and kinematic bicycle model, sharing
an identical longitudinal speed law so the comparison isolates *steering*
(full report: [`docs_en/comparison_report.md`](docs_en/comparison_report.md)):

| Algorithm | CTE RMS [m] | CTE max [m] | Heading RMS [deg] | Steer rate RMS [deg/s] | Compute [µs/cycle] |
|-----------|-------------|-------------|-------------------|------------------------|--------------------|
| Pure Pursuit | 0.038 | 0.198 | 0.235 | **1.6** | **0.35** |
| Stanley | 0.040 | 0.141 | 0.151 | 14.3 | 0.37 |
| MPC | **0.008** | **0.030** | **0.115** | 46.6 | 1.52 |
| MPPI | 0.036 | 0.138 | 0.384 | 174.8 | 568 |

On this kinematic test **MPC achieves the lowest cross-track error** — about
5× tighter than the others — while **Pure Pursuit gives the smoothest steering**
at the lowest compute cost. MPPI pays ~1600× the per-cycle cost of the geometric
methods for its sampling search.

## Metrics

For each run the simulator reports:

| Metric | Meaning |
|--------|---------|
| `cte_rms` / `cte_max` | RMS / peak cross-track error [m] |
| `heading_rms` | RMS heading error [deg] |
| `steer_rms` | RMS steering angle [deg] |
| `steer_rate_rms` | RMS steering rate [deg/s] — actuation smoothness |
| `avg_speed` | mean speed [km/h] |
| `compute_us_per_cycle` | controller wall-clock time per cycle [µs] |

## ACC cut-in following demo

Beyond pure path tracking, the repository includes an **Adaptive Cruise
Control (ACC) cut-in scenario** (`src/cutin_demo.cpp` +
`include/path_tracking/acc_controller.hpp`). The ego vehicle cruises along the
reference route; partway through, a slower lead vehicle cuts in ahead. The ACC
controller wraps Pure Pursuit lateral control with a gap-following longitudinal
law:

```
gap_desired = max(min_gap, time_gap * v_ego)
a = kp_dv * (v_lead - v_ego) + kp_gap * (gap - gap_desired)
a = clamp(a, -decel_max, accel_max)
```

producing three visible phases: free cruise → hard brake on the cut-in →
steady gap-following at the lead speed.

```bash
# Build (the cutin_demo target is part of the normal CMake build)
cmake --build build --target cutin_demo

# Run — writes cutin_trace.csv
./build/cutin_demo [route.csv] [out_dir]

# Visualise / animate
python tools/plot_cutin_demo.py    cutin_trace.csv
python tools/animate_cutin_demo.py cutin_trace.csv --save cutin.gif
```

## Tests

A header-only regression test (no external framework) is registered with CTest:

```bash
cmake --build build --target path_tracking_tests
ctest --test-dir build --output-on-failure
```

It checks `cross_track_error` sign/magnitude, that the windowed `nearest_index`
search agrees with a full scan, that `assign_speed_profile` lowers the reference
speed as curvature grows, and that Pure Pursuit converges to a straight line
from a 1 m lateral offset.

## Reference route data

[`data/reference_route.csv`](data/reference_route.csv) is a synthetic route
(~1.0 km) built the way real road geometry is laid out: straights joined to
constant-radius arcs by clothoid transitions, with bends of R = 200, 30, 150
and 25 m. It is generated by
[`tools/generate_reference_route.py`](tools/generate_reference_route.py), so the
geometry can be changed by editing the curvature profile at the top of that
script and re-running it. No third-party map data is involved. See
[`data/README.md`](data/README.md) for the column format.

## License

Apache License 2.0 — see the `SPDX-License-Identifier` header in each source file.
