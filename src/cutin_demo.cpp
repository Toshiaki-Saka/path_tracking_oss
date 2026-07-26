// SPDX-License-Identifier: Apache-2.0
//
// cutin_demo.cpp - Cut-in vehicle following control demonstration.
//
// Scenario:
//   1. Ego vehicle follows the reference path at normal speed (~29 km/h).
//   2. At arc-length CUTIN_TRIGGER_S a slower lead vehicle appears
//      CUTIN_OFFSET m ahead at LEAD_SPEED m/s, simulating a cut-in.
//   3. The ACC controller detects the lead, applies hard braking, then settles
//      into stable gap-following at the lead vehicle's speed.
//
// Three phases are clearly visible in the output:
//   Phase 0  free cruise  - ego follows path reference speed
//   Phase 1  hard brake   - ACC reacts to cut-in, closes the speed gap
//   Phase 2  follow       - ego matches lead speed, maintains desired gap
//
// Output:
//   <out_dir>/cutin_trace.csv  - per-step trace (ego + lead states)
//
// Usage:
//   cutin_demo [route.csv] [out_dir]
//
#include "path_tracking/acc_controller.hpp"
#include "path_tracking/path_io.hpp"
#include "path_tracking/vehicle.hpp"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace path_tracking;

// ---------------------------------------------------------------------------
// Scenario parameters
// ---------------------------------------------------------------------------
static constexpr double kDt               = 0.05;   // control cycle [s]
static constexpr double kCutinTriggerS    = 200.0;  // cut-in when ego_s >= this [m]
static constexpr double kCutinOffset      =  15.0;  // lead appears this far ahead [m]
static constexpr double kCutinLateralOff  =   3.5;  // initial lateral offset from path [m]
static constexpr double kCutinMergeDist   =  40.0;  // lane-change completes over this arc [m]
static constexpr double kLeadSpeed        =   3.0;  // lead vehicle speed [m/s] (~10.8 km/h)
static constexpr double kDemoEndS         = 700.0;  // stop the demo at this arc-length [m]
static constexpr int    kMaxSteps         = 100000;

// Find the path index whose arc-length is closest to target_s.
static std::size_t idx_at_s(const Path& path, double target_s) noexcept {
    const std::size_t n = path.size();
    if (n == 0) return 0;
    if (target_s <= path[0].s) return 0;
    if (target_s >= path[n - 1].s) return n - 1;
    std::size_t lo = 0, hi = n - 1;
    while (lo + 1 < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (path[mid].s < target_s) lo = mid; else hi = mid;
    }
    return (std::abs(path[hi].s - target_s) < std::abs(path[lo].s - target_s)) ? hi : lo;
}

// ---------------------------------------------------------------------------
// Per-step trace record
// ---------------------------------------------------------------------------
struct DemoRow {
    int    step;
    double time_s;
    double x_m, y_m, yaw_rad;
    double ego_speed_mps;
    double ego_s_m;
    double steer_deg;
    double accel_mps2;
    double cte_m;
    double lead_speed_mps;
    double lead_s_m;
    double lead_x_m, lead_y_m, lead_yaw_rad;
    double lead_lat_m;     // lateral offset of lead from reference path [m]
    double gap_m;          // -1 when no lead vehicle yet
    int    acc_mode;       // 0 = free cruise, 1 = gap-following
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void write_csv(const std::string& filepath,
                      const std::vector<DemoRow>& rows) {
    std::ofstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "  warning: cannot write " << filepath << "\n";
        return;
    }
    f << "step,time_s,x_m,y_m,yaw_rad,"
         "ego_speed_kmh,ego_s_m,steer_deg,accel_mps2,cte_m,"
         "lead_speed_kmh,lead_s_m,"
         "lead_x_m,lead_y_m,lead_yaw_rad,lead_lat_m,"
         "gap_m,acc_mode\n";
    f << std::fixed << std::setprecision(5);
    for (const auto& r : rows) {
        f << r.step << ','
          << r.time_s << ','
          << r.x_m << ',' << r.y_m << ',' << r.yaw_rad << ','
          << r.ego_speed_mps * 3.6 << ','
          << r.ego_s_m << ','
          << r.steer_deg << ','
          << r.accel_mps2 << ','
          << r.cte_m << ','
          << r.lead_speed_mps * 3.6 << ','
          << r.lead_s_m << ','
          << r.lead_x_m << ',' << r.lead_y_m << ',' << r.lead_yaw_rad << ','
          << r.lead_lat_m << ','
          << r.gap_m << ','
          << r.acc_mode << '\n';
    }
    std::cout << "  wrote " << filepath
              << "  (" << rows.size() << " rows)\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    const std::string route_csv =
        (argc > 1) ? argv[1] : "data/reference_route.csv";
    std::string out_dir = (argc > 2) ? argv[2] : ".";
    if (!out_dir.empty() &&
        out_dir.back() != '/' && out_dir.back() != '\\')
        out_dir += '/';

    // ---- Load reference path -----------------------------------------------
    Path path;
    try {
        SpeedProfileConfig spd;
        path = load_path_csv(route_csv, spd);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\n=== Cut-in following control demo ===\n\n"
              << "Route       : " << route_csv << "\n"
              << "  points    : " << path.size() << "\n"
              << "  length    : " << std::fixed << std::setprecision(1)
              << path.total_length() << " m\n\n"
              << "Cut-in at   : s >= " << kCutinTriggerS << " m\n"
              << "Lead offset : " << kCutinOffset << " m ahead\n"
              << "Lead speed  : " << kLeadSpeed * 3.6 << " km/h\n"
              << "Demo end    : s = " << kDemoEndS << " m\n\n";

    // ---- Controller setup --------------------------------------------------
    PurePursuitConfig lat_cfg;
    lat_cfg.wheelbase = 2.7;

    AccConfig acc_cfg;
    acc_cfg.time_gap        = 2.0;   // [s]
    acc_cfg.min_gap         = 5.0;   // [m]
    acc_cfg.kp_dv           = 1.2;
    acc_cfg.kp_gap          = 0.4;
    acc_cfg.detection_range = 80.0;
    acc_cfg.accel_max       = 1.5;
    acc_cfg.decel_max       = 4.0;

    AccController ctrl(lat_cfg, acc_cfg);
    ctrl.reset();

    // ---- Vehicle init ------------------------------------------------------
    Vehicle ego(2.7, kDt);
    {
        VehicleState s0;
        s0.x   = path[0].x;
        s0.y   = path[0].y;
        s0.yaw = path[0].yaw;
        s0.v   = path[0].v_ref * 0.4;   // start at 40% of reference speed
        ego.set_state(s0);
    }

    // ---- Cut-in state ------------------------------------------------------
    LeadVehicle lead{};
    bool   cutin_done    = false;
    double cutin_t       = -1.0;
    double cutin_ego_s   = -1.0;
    double cutin_lead_s0 = 0.0;  // lead.s at the moment of cut-in
    double cur_lead_lat  = 0.0;  // current lateral offset of lead from path

    // ---- Simulation loop ---------------------------------------------------
    std::vector<DemoRow> trace;
    trace.reserve(8000);
    std::size_t hint = 0;

    for (int step = 0; step < kMaxSteps; ++step) {
        const double      t  = step * kDt;
        const VehicleState& st = ego.state();

        hint = nearest_index(path, st.x, st.y, hint, 400);
        const double ego_s = path[hint].s;

        // ---- Cut-in trigger ------------------------------------------------
        if (!cutin_done && ego_s >= kCutinTriggerS) {
            lead.active   = true;
            lead.xy_valid = true;
            lead.s        = ego_s + kCutinOffset;
            lead.v        = kLeadSpeed;
            cutin_done    = true;
            cutin_t       = t;
            cutin_ego_s   = ego_s;
            cutin_lead_s0 = lead.s;

            std::cout << "*** CUT-IN  t=" << std::setprecision(2) << t
                      << " s   ego_s=" << std::setprecision(1) << ego_s
                      << " m   lead_s=" << lead.s
                      << " m   lead=" << lead.v * 3.6 << " km/h"
                      << "   ego=" << st.v * 3.6 << " km/h ***\n";
        }

        // ---- Advance lead vehicle and update its world-frame XY -------------
        if (lead.active) {
            lead.s += lead.v * kDt;

            // Cosine taper: lateral offset goes from kCutinLateralOff → 0
            // over kCutinMergeDist metres of arc-length (smooth lane change).
            const double prog = std::min(1.0,
                (lead.s - cutin_lead_s0) / kCutinMergeDist);
            cur_lead_lat = kCutinLateralOff * 0.5 * (1.0 + std::cos(kPi * prog));

            const std::size_t li = idx_at_s(path, lead.s);
            const auto& lp = path[li];
            // Left-normal of path tangent: (-sin(yaw), cos(yaw))
            lead.x   = lp.x + (-std::sin(lp.yaw)) * cur_lead_lat;
            lead.y   = lp.y +   std::cos(lp.yaw)  * cur_lead_lat;
            lead.yaw = lp.yaw;
        }

        // ---- Compute control command ----------------------------------------
        ctrl.set_lead(lead);
        const ControlCommand cmd = ctrl.compute(path, st, kDt);

        // ---- Record trace --------------------------------------------------
        const double cte = cross_track_error(path, hint, st.x, st.y);
        const double gap = lead.active ? (lead.s - ego_s) : -1.0;
        const bool follow = lead.active && gap > 0.1
                         && gap < acc_cfg.detection_range;

        DemoRow row;
        row.step           = step;
        row.time_s         = t;
        row.x_m            = st.x;
        row.y_m            = st.y;
        row.yaw_rad        = st.yaw;
        row.ego_speed_mps  = st.v;
        row.ego_s_m        = ego_s;
        row.steer_deg      = cmd.steer * 180.0 / kPi;
        row.accel_mps2     = cmd.accel;
        row.cte_m          = cte;
        row.lead_speed_mps = lead.active ? lead.v : 0.0;
        row.lead_s_m       = lead.s;
        row.lead_x_m       = lead.active ? lead.x : 0.0;
        row.lead_y_m       = lead.active ? lead.y : 0.0;
        row.lead_yaw_rad   = lead.active ? lead.yaw : 0.0;
        row.lead_lat_m     = lead.active ? cur_lead_lat : 0.0;
        row.gap_m          = gap;
        row.acc_mode       = follow ? 1 : 0;
        trace.push_back(row);

        // ---- Advance vehicle -----------------------------------------------
        ego.step(cmd.accel, cmd.steer);

        // ---- End condition -------------------------------------------------
        if (ego_s >= kDemoEndS || ego_s >= path.total_length() - 5.0) break;
    }

    // ---- Summary -----------------------------------------------------------
    const auto& last = trace.back();
    std::cout << "\n--- Demo summary ---\n"
              << "  Steps       : " << trace.size() << "\n"
              << "  Sim time    : " << std::setprecision(1)
              << last.time_s << " s\n"
              << "  Ego s       : " << last.ego_s_m << " m\n";
    if (cutin_done) {
        std::cout << "  Cut-in at t : " << cutin_t << " s  (s="
                  << cutin_ego_s << " m)\n";
        std::cout << "  Final gap   : " << std::setprecision(2)
                  << last.gap_m << " m\n";
        std::cout << "  Final ego v : " << last.ego_speed_mps * 3.6
                  << " km/h  (lead=" << kLeadSpeed * 3.6 << " km/h)\n";
    }

    // ---- Write CSV ---------------------------------------------------------
    write_csv(out_dir + "cutin_trace.csv", trace);

    std::cout << "\nPlot with:\n"
              << "  python tools/plot_cutin_demo.py "
              << out_dir << " data/reference_route.csv\n";
    return 0;
}
