// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// acc_controller.hpp - Adaptive Cruise Control (ACC) / car-following controller.
//
// Wraps Pure Pursuit lateral control with a gap-following longitudinal law:
//
//   gap_desired = max(min_gap, time_gap * v_ego)
//   a = kp_dv * (v_lead - v_ego) + kp_gap * (gap - gap_desired)
//   a = clamp(a, -decel_max, accel_max)
//
// Falls back to path reference speed tracking when no lead vehicle is present
// or the lead is beyond detection_range.
//
#include "controller.hpp"
#include "path_io.hpp"
#include "pure_pursuit.hpp"
#include <cmath>

namespace path_tracking {

// Lead vehicle state: arc-length + actual world-frame pose.
struct LeadVehicle {
    double s{0.0};          // arc-length position on reference path [m]
    double v{0.0};          // speed [m/s]
    double x{0.0};          // world x [m]
    double y{0.0};          // world y [m]
    double yaw{0.0};        // heading [rad]
    bool   active{false};   // false until cut-in fires
    bool   xy_valid{false}; // true once x,y,yaw are current
};

struct AccConfig {
    double time_gap{2.0};         // desired time headway [s]
    double min_gap{5.0};          // absolute minimum gap [m]
    double kp_dv{1.2};            // gain on relative speed (v_lead - v_ego)
    double kp_gap{0.4};           // gain on gap error
    double detection_range{80.0}; // ignore lead beyond this distance [m]
    double accel_max{1.5};        // [m/s^2]
    double decel_max{4.0};        // [m/s^2] (magnitude)
};

// Compute one longitudinal acceleration command under ACC.
// Returns the path-speed-tracking acceleration when no lead is present.
[[nodiscard]] inline double acc_accel(
        double ego_s, double ego_v,
        const LeadVehicle& lead,
        const AccConfig& acc,
        double path_v_ref,
        const LongitudinalConfig& lon) {

    if (lead.active) {
        const double gap = lead.s - ego_s;
        if (gap > 0.1 && gap < acc.detection_range) {
            const double gap_des = std::max(acc.min_gap, acc.time_gap * ego_v);
            const double a = acc.kp_dv  * (lead.v - ego_v)
                           + acc.kp_gap * (gap - gap_des);
            return clampd(a, -acc.decel_max, acc.accel_max);
        }
    }
    return longitudinal_accel(path_v_ref, ego_v, lon);
}

// ACC controller: Pure Pursuit steering + ACC longitudinal.
// Call set_lead() each simulation step before compute().
class AccController final : public Controller {
public:
    explicit AccController(PurePursuitConfig lat = {},
                           AccConfig         acc = {})
        : lat_(lat), acc_(acc) {}

    [[nodiscard]] std::string name() const override { return "ACC"; }

    void reset() override { last_idx_ = 0; lead_ = {}; }

    void set_lead(const LeadVehicle& lead) noexcept { lead_ = lead; }
    [[nodiscard]] const LeadVehicle& lead() const noexcept { return lead_; }
    [[nodiscard]] const AccConfig&   acc_cfg() const noexcept { return acc_; }

    [[nodiscard]] ControlCommand
    compute(const Path& path, const VehicleState& s, double /*dt*/) override {
        ControlCommand cmd{};

        const std::size_t near = nearest_index(path, s.x, s.y, last_idx_, 300);
        last_idx_ = near;

        const double gap = lead_.active ? (lead_.s - path[near].s) : -1.0;
        const bool in_follow = lead_.active && lead_.xy_valid
                            && gap > 0.1 && gap < acc_.detection_range;

        if (in_follow) {
            // --- Trajectory following: steer toward lead vehicle XY ---
            const double dx    = lead_.x - s.x;
            const double dy    = lead_.y - s.y;
            const double alpha = wrap_to_pi(std::atan2(dy, dx) - s.yaw);
            const double dist  = std::max(std::hypot(dx, dy), 1.0);
            cmd.steer = clampd(
                std::atan2(2.0 * lat_.wheelbase * std::sin(alpha), dist),
                -lat_.steer_limit, lat_.steer_limit);
        } else {
            // --- Pure Pursuit on reference path ---
            const double ld = clampd(lat_.ld_gain * s.v + lat_.ld_min,
                                     lat_.ld_min, lat_.ld_max);
            std::size_t tgt = near;
            const double arc0 = path[near].s;
            while (tgt + 1 < path.size() && (path[tgt].s - arc0) < ld) ++tgt;

            const double dx    = path[tgt].x - s.x;
            const double dy    = path[tgt].y - s.y;
            const double alpha = wrap_to_pi(std::atan2(dy, dx) - s.yaw);
            const double ld_eff = std::max(std::hypot(dx, dy), 1e-3);
            cmd.steer = clampd(
                std::atan2(2.0 * lat_.wheelbase * std::sin(alpha), ld_eff),
                -lat_.steer_limit, lat_.steer_limit);
        }

        // --- ACC longitudinal (unchanged) ---
        cmd.accel = acc_accel(path[near].s, s.v, lead_, acc_,
                              path[near].v_ref, lat_.lon);
        return cmd;
    }

private:
    PurePursuitConfig lat_;
    AccConfig         acc_;
    LeadVehicle       lead_{};
    std::size_t       last_idx_{0};
};

} // namespace path_tracking
