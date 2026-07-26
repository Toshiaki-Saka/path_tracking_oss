// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// pure_pursuit.hpp - Pure Pursuit path-tracking controller.
//
// Geometric tracker (1990s, classic): chase a look-ahead point on the path.
//   delta = atan2( 2 * L * sin(alpha), ld )
// where L is the wheelbase, ld the look-ahead distance and alpha the angle
// between the vehicle heading and the look-ahead point.
//
//   + Extremely simple, near-zero compute cost, stable at low speed.
//   - Tends to cut corners at high speed; does not act on lateral error
//     directly; the look-ahead distance must be tuned per vehicle.
//
#include "controller.hpp"
#include "path_io.hpp"
#include <cmath>

namespace path_tracking {

struct PurePursuitConfig {
    double wheelbase{2.7};      // [m]
    double ld_gain{0.6};        // look-ahead = ld_gain * v + ld_min
    double ld_min{3.0};         // minimum look-ahead distance [m]
    double ld_max{20.0};        // maximum look-ahead distance [m]
    double steer_limit{0.6};    // |steer| limit [rad]
    LongitudinalConfig lon{};
};

class PurePursuitController final : public Controller {
public:
    explicit PurePursuitController(PurePursuitConfig cfg = {}) : cfg_(cfg) {}

    [[nodiscard]] std::string name() const override { return "PurePursuit"; }

    void reset() override { last_idx_ = 0; }

    [[nodiscard]] ControlCommand
    compute(const Path& path, const VehicleState& s, double /*dt*/) override {
        ControlCommand cmd{};

        // Nearest path point (used for the speed reference).
        const std::size_t near = nearest_index(path, s.x, s.y, last_idx_, 300);
        last_idx_ = near;

        // Look-ahead distance grows with speed.
        const double ld =
            clampd(cfg_.ld_gain * s.v + cfg_.ld_min, cfg_.ld_min, cfg_.ld_max);

        // Walk forward along the path until we are at least `ld` ahead.
        std::size_t tgt = near;
        const double s0 = path[near].s;
        while (tgt + 1 < path.size() && (path[tgt].s - s0) < ld) {
            ++tgt;
        }

        // Angle from the vehicle to the look-ahead point.
        const double dx = path[tgt].x - s.x;
        const double dy = path[tgt].y - s.y;
        const double alpha = wrap_to_pi(std::atan2(dy, dx) - s.yaw);
        const double ld_eff = std::max(std::hypot(dx, dy), 1e-3);

        const double steer =
            std::atan2(2.0 * cfg_.wheelbase * std::sin(alpha), ld_eff);
        cmd.steer = clampd(steer, -cfg_.steer_limit, cfg_.steer_limit);

        cmd.accel = longitudinal_accel(path[near].v_ref, s.v, cfg_.lon);
        return cmd;
    }

private:
    PurePursuitConfig cfg_;
    std::size_t       last_idx_{0};
};

} // namespace path_tracking
