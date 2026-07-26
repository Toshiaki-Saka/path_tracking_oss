// SPDX-License-Identifier: Apache-2.0
//
// main.cpp - Path-tracking algorithm comparison benchmark.
//
// Loads a reference path from CSV, runs four path-tracking controllers
// (Pure Pursuit / Stanley / MPC / MPPI) around it with an identical
// kinematic vehicle model, writes a per-step trace CSV for each, and prints
// a side-by-side performance comparison.
//
// Usage:
//   path_tracking_compare [route.csv] [output_dir]
//
//   route.csv  : reference path (default: data/reference_route.csv)
//   output_dir : where trace CSVs are written (default: current directory)
//
#include "path_tracking/mpc.hpp"
#include "path_tracking/mppi.hpp"
#include "path_tracking/path_io.hpp"
#include "path_tracking/pure_pursuit.hpp"
#include "path_tracking/simulator.hpp"
#include "path_tracking/stanley.hpp"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace path_tracking;

namespace {

void write_trace_csv(const std::string& path, const RunResult& r) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "  warning: cannot write " << path << "\n";
        return;
    }
    f << "step,time_s,x_m,y_m,yaw_rad,speed_kmh,steer_deg,accel_mps2,"
         "cte_m,heading_err_deg,map_id\n";
    f << std::fixed << std::setprecision(5);
    for (const auto& t : r.trace) {
        f << t.step << ','
          << t.time_s << ','
          << t.x << ',' << t.y << ',' << t.yaw << ','
          << t.v * 3.6 << ','
          << t.steer * 180.0 / kPi << ','
          << t.accel << ','
          << t.cte << ','
          << t.heading_err * 180.0 / kPi << ','
          << t.map_id << '\n';
    }
}

void write_summary_csv(const std::string& path,
                       const std::vector<RunMetrics>& all) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "  warning: cannot write " << path << "\n";
        return;
    }
    f << "algorithm,completed,steps,sim_time_s,cte_rms_m,cte_max_m,"
         "cte_mean_abs_m,heading_rms_deg,steer_rms_deg,steer_rate_rms_dps,"
         "avg_speed_kmh,final_pos_err_m,compute_us_per_cycle\n";
    f << std::fixed << std::setprecision(5);
    for (const auto& m : all) {
        f << m.algorithm << ','
          << (m.completed ? 1 : 0) << ','
          << m.steps << ','
          << m.sim_time_s << ','
          << m.cte_rms << ','
          << m.cte_max << ','
          << m.cte_mean_abs << ','
          << m.heading_rms * 180.0 / kPi << ','
          << m.steer_rms * 180.0 / kPi << ','
          << m.steer_rate_rms * 180.0 / kPi << ','
          << m.avg_speed_kmh << ','
          << m.final_pos_err << ','
          << m.compute_us_avg << '\n';
    }
}

void print_comparison(const std::vector<RunMetrics>& all) {
    auto rule = [] {
        std::cout << "  " << std::string(98, '-') << "\n";
    };
    std::cout << "\n=== Performance comparison ===\n\n";
    std::cout << "  " << std::left
              << std::setw(14) << "Algorithm"
              << std::right
              << std::setw(10) << "CTE RMS"
              << std::setw(10) << "CTE max"
              << std::setw(11) << "Head RMS"
              << std::setw(11) << "Steer RMS"
              << std::setw(12) << "SteerRate"
              << std::setw(11) << "AvgSpeed"
              << std::setw(11) << "Compute"
              << std::setw(8)  << "Done"
              << "\n";
    std::cout << "  " << std::left
              << std::setw(14) << ""
              << std::right
              << std::setw(10) << "[m]"
              << std::setw(10) << "[m]"
              << std::setw(11) << "[deg]"
              << std::setw(11) << "[deg]"
              << std::setw(12) << "[deg/s]"
              << std::setw(11) << "[km/h]"
              << std::setw(11) << "[us/cyc]"
              << std::setw(8)  << ""
              << "\n";
    rule();
    std::cout << std::fixed;
    for (const auto& m : all) {
        std::cout << "  " << std::left
                  << std::setw(14) << m.algorithm
                  << std::right << std::setprecision(4)
                  << std::setw(10) << m.cte_rms
                  << std::setw(10) << m.cte_max
                  << std::setprecision(3)
                  << std::setw(11) << m.heading_rms * 180.0 / kPi
                  << std::setw(11) << m.steer_rms * 180.0 / kPi
                  << std::setw(12) << m.steer_rate_rms * 180.0 / kPi
                  << std::setprecision(2)
                  << std::setw(11) << m.avg_speed_kmh
                  << std::setw(11) << m.compute_us_avg
                  << std::setw(8)  << (m.completed ? "yes" : "NO")
                  << "\n";
    }
    rule();

    // Highlight the best (lowest) tracking-accuracy result.
    std::size_t best = 0;
    for (std::size_t i = 1; i < all.size(); ++i) {
        if (all[i].cte_rms < all[best].cte_rms) best = i;
    }
    std::cout << "\n  Best tracking accuracy (lowest CTE RMS): "
              << all[best].algorithm << "  ("
              << std::setprecision(4) << all[best].cte_rms << " m)\n\n";
}

} // namespace

int main(int argc, char* argv[]) {
    const std::string route_csv =
        (argc > 1) ? argv[1] : "data/reference_route.csv";
    std::string out_dir = (argc > 2) ? argv[2] : ".";
    if (!out_dir.empty() && out_dir.back() != '/') out_dir += '/';

    // ---- Load the reference path -----------------------------------------
    Path path;
    try {
        SpeedProfileConfig spd;   // curvature-limited speed profile
        path = load_path_csv(route_csv, spd);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Loaded route: " << route_csv << "\n"
              << "  points  : " << path.size() << "\n"
              << "  length  : " << std::fixed << std::setprecision(1)
              << path.total_length() << " m\n";

    // ---- Common simulation configuration ---------------------------------
    SimConfig sim;
    sim.dt        = 0.01;
    sim.wheelbase = 2.7;

    // ---- Build the four controllers --------------------------------------
    // Each controller shares the same wheelbase and longitudinal law.
    std::vector<std::unique_ptr<Controller>> controllers;

    {
        PurePursuitConfig c; c.wheelbase = sim.wheelbase;
        controllers.push_back(std::make_unique<PurePursuitController>(c));
    }
    {
        StanleyConfig c; c.wheelbase = sim.wheelbase;
        controllers.push_back(std::make_unique<StanleyController>(c));
    }
    {
        MpcConfig c; c.wheelbase = sim.wheelbase;
        controllers.push_back(std::make_unique<MpcController>(c));
    }
    {
        MppiConfig c; c.wheelbase = sim.wheelbase;
        controllers.push_back(std::make_unique<MppiController>(c));
    }

    // ---- Run each controller and collect results -------------------------
    std::vector<RunMetrics> summary;
    for (auto& ctrl : controllers) {
        std::cout << "\nRunning " << ctrl->name() << " ...\n";
        const RunResult r = simulate(*ctrl, path, sim);

        const std::string trace_path =
            out_dir + "trace_" + ctrl->name() + ".csv";
        write_trace_csv(trace_path, r);

        std::cout << "  steps=" << r.metrics.steps
                  << "  sim_time=" << std::fixed << std::setprecision(1)
                  << r.metrics.sim_time_s << " s"
                  << "  CTE_rms=" << std::setprecision(4)
                  << r.metrics.cte_rms << " m"
                  << "  -> " << trace_path << "\n";

        summary.push_back(r.metrics);
    }

    // ---- Summary ---------------------------------------------------------
    const std::string summary_path = out_dir + "comparison_summary.csv";
    write_summary_csv(summary_path, summary);
    print_comparison(summary);
    std::cout << "  Summary CSV: " << summary_path << "\n";

    return 0;
}
