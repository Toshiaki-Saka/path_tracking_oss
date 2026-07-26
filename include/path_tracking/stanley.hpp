// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// stanley.hpp - Stanley path-tracking controller.
//
// Front-axle based geometric tracker (2005 DARPA Grand Challenge winner).
// The steering command corrects two errors at the front axle:
//   delta = heading_error + atan2( k * cross_track_error, k_soft + v )
//
//   + Acts on lateral error directly; stable at highway speeds; provably
//     converges asymptotically for the kinematic model.
//   - Can be over-sensitive at very low speed; uses a simple vehicle model.
//
// A small phase-advance (lead) filter on the steering command is included
// to suppress oscillation on sharp curves; set lead_gain = 0 to disable.
//
#include "controller.hpp"
#include "path_io.hpp"
#include <cmath>

namespace path_tracking {

struct StanleyConfig {
    double wheelbase{2.7};      // [m]
    double k_cross{2.5};        // cross-track error gain
    double k_soft{1.0};         // softening term [m/s] (avoids /0 at v=0)
    double k_heading{1.0};      // heading-error weight
    double steer_limit{0.6};    // |steer| limit [rad]
    double steer_rate_limit{4.0}; // |d(steer)/dt| limit [rad/s]
    double lead_gain{0.15};     // phase-advance amount (0 = off)
    LongitudinalConfig lon{};
};

class StanleyController final : public Controller {
public:
    explicit StanleyController(StanleyConfig cfg = {}) : cfg_(cfg) {}

    [[nodiscard]] std::string name() const override { return "Stanley"; }

    void reset() override {
        last_idx_   = 0;
        prev_steer_ = 0.0;
        lead_state_ = 0.0;
    }

    [[nodiscard]] ControlCommand
    compute(const Path& path, const VehicleState& s, double dt) override {
        ControlCommand cmd{};

        // Front-axle position (Stanley references the front axle, not the CoG).
        const double fx = s.x + cfg_.wheelbase * std::cos(s.yaw);
        const double fy = s.y + cfg_.wheelbase * std::sin(s.yaw);

        const std::size_t near = nearest_index(path, fx, fy, last_idx_, 300);
        last_idx_ = near;

        // Heading error: path tangent minus vehicle heading.
        const double heading_err = wrap_to_pi(path[near].yaw - s.yaw);

        // Cross-track error at the front axle.
        // cross_track_error() is positive when the vehicle is to the LEFT of
        // the path tangent; correcting it requires steering to the RIGHT,
        // hence the cross-track term is SUBTRACTED.
        const double e_cross = cross_track_error(path, near, fx, fy);

        // Stanley steering law:  delta = heading_err - atan2(k*e_cross, k_soft+v)
        const double cross_term =
            std::atan2(cfg_.k_cross * e_cross, cfg_.k_soft + s.v);
        double steer = cfg_.k_heading * heading_err - cross_term;

        // Optional phase-advance (lead) filter to damp curve oscillation.
        if (cfg_.lead_gain > 0.0) {
            const double lead = steer + cfg_.lead_gain * (steer - lead_state_);
            lead_state_ = steer;
            steer = lead;
        }

        steer = clampd(steer, -cfg_.steer_limit, cfg_.steer_limit);

        // Steering-rate limit for smoother, more realistic actuation.
        const double max_step = cfg_.steer_rate_limit * dt;
        steer = clampd(steer, prev_steer_ - max_step, prev_steer_ + max_step);
        prev_steer_ = steer;

        cmd.steer = steer;
        cmd.accel = longitudinal_accel(path[near].v_ref, s.v, cfg_.lon);
        return cmd;
    }

private:
    StanleyConfig cfg_;
    std::size_t   last_idx_{0};
    double        prev_steer_{0.0};
    double        lead_state_{0.0};
};

} // namespace path_tracking
