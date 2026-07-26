// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// simulator.hpp - Closed-loop simulation and performance metrics.
//
// Drives a Controller around a Path using the kinematic Vehicle model and
// records a per-step trace plus aggregate metrics used for comparison.
//
#include "controller.hpp"
#include "path_io.hpp"
#include "types.hpp"
#include "vehicle.hpp"
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace path_tracking {

// One recorded simulation step.
struct TraceRow {
    int    step{0};
    double time_s{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double v{0.0};          // [m/s]
    double steer{0.0};      // [rad]
    double accel{0.0};      // [m/s^2]
    double cte{0.0};        // signed cross-track error [m]
    double heading_err{0.0};// [rad]
    std::size_t map_id{0};  // nearest path index
};

// Aggregate performance metrics for one run.
struct RunMetrics {
    std::string algorithm;
    bool   completed{false};     // reached the route end
    int    steps{0};
    double sim_time_s{0.0};      // simulated duration
    double compute_ms_total{0.0};// wall-clock controller time
    double compute_us_avg{0.0};  // per-cycle controller time

    double cte_rms{0.0};         // RMS cross-track error [m]
    double cte_max{0.0};         // peak |cross-track error| [m]
    double cte_mean_abs{0.0};    // mean |cross-track error| [m]
    double heading_rms{0.0};     // RMS heading error [rad]
    double steer_rms{0.0};       // RMS steering angle [rad]
    double steer_rate_rms{0.0};  // RMS steering rate [rad/s] (smoothness)
    double avg_speed_kmh{0.0};   // mean speed [km/h]
    double final_pos_err{0.0};   // distance from last path point [m]
};

struct SimConfig {
    double dt{0.01};             // control cycle [s]
    double wheelbase{2.7};       // [m]
    int    max_steps{200000};    // safety cap
    double start_speed_frac{0.3};// initial speed as fraction of v_ref[0]
    double finish_margin{2.0};   // distance-to-end that counts as "arrived" [m]
};

struct RunResult {
    RunMetrics            metrics;
    std::vector<TraceRow> trace;
};

// Run one closed-loop simulation of `ctrl` on `path`.
inline RunResult simulate(Controller& ctrl, const Path& path,
                          const SimConfig& cfg) {
    RunResult result;
    result.metrics.algorithm = ctrl.name();

    Vehicle veh(cfg.wheelbase, cfg.dt);
    VehicleState s0;
    s0.x   = path[0].x;
    s0.y   = path[0].y;
    s0.yaw = path[0].yaw;
    s0.v   = path[0].v_ref * cfg.start_speed_frac;
    veh.set_state(s0);

    ctrl.reset();

    double sum_cte2 = 0.0, sum_heading2 = 0.0, sum_steer2 = 0.0;
    double sum_abs_cte = 0.0, sum_v = 0.0, sum_dsteer2 = 0.0;
    double prev_steer = 0.0;
    std::size_t hint = 0;

    const std::size_t end_idx = path.size() - 1;
    const double end_s = path.total_length();

    double compute_total_ns = 0.0;

    int step = 0;
    for (; step < cfg.max_steps; ++step) {
        const VehicleState& st = veh.state();

        const auto t0 = std::chrono::steady_clock::now();
        const ControlCommand cmd = ctrl.compute(path, st, cfg.dt);
        const auto t1 = std::chrono::steady_clock::now();
        compute_total_ns +=
            std::chrono::duration<double, std::nano>(t1 - t0).count();

        // Metrics relative to the nearest path point.
        hint = nearest_index(path, st.x, st.y, hint, 400);
        const double cte = cross_track_error(path, hint, st.x, st.y);
        const double herr = wrap_to_pi(path[hint].yaw - st.yaw);

        TraceRow row;
        row.step        = step;
        row.time_s      = step * cfg.dt;
        row.x           = st.x;
        row.y           = st.y;
        row.yaw         = st.yaw;
        row.v           = st.v;
        row.steer       = cmd.steer;
        row.accel       = cmd.accel;
        row.cte         = cte;
        row.heading_err = herr;
        row.map_id      = hint;
        result.trace.push_back(row);

        sum_cte2     += cte * cte;
        sum_abs_cte  += std::abs(cte);
        sum_heading2 += herr * herr;
        sum_steer2   += cmd.steer * cmd.steer;
        sum_v        += st.v;
        const double dsteer = (cmd.steer - prev_steer) / cfg.dt;
        sum_dsteer2  += dsteer * dsteer;
        prev_steer    = cmd.steer;

        result.metrics.cte_max =
            std::max(result.metrics.cte_max, std::abs(cte));

        // Advance the vehicle.
        veh.step(cmd.accel, cmd.steer);

        // Termination: arrived near the final path point.
        if (path[hint].s >= end_s - cfg.finish_margin && hint >= end_idx - 3) {
            result.metrics.completed = true;
            ++step;
            break;
        }
    }

    const int n = std::max(step, 1);
    RunMetrics& m = result.metrics;
    m.steps            = step;
    m.sim_time_s       = step * cfg.dt;
    m.compute_ms_total = compute_total_ns / 1e6;
    m.compute_us_avg   = compute_total_ns / 1e3 / n;
    m.cte_rms          = std::sqrt(sum_cte2 / n);
    m.cte_mean_abs     = sum_abs_cte / n;
    m.heading_rms      = std::sqrt(sum_heading2 / n);
    m.steer_rms        = std::sqrt(sum_steer2 / n);
    m.steer_rate_rms   = std::sqrt(sum_dsteer2 / n);
    m.avg_speed_kmh    = (sum_v / n) * 3.6;

    const VehicleState& sf = veh.state();
    m.final_pos_err = std::hypot(sf.x - path[end_idx].x,
                                 sf.y - path[end_idx].y);
    return result;
}

} // namespace path_tracking
