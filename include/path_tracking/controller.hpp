// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// controller.hpp - Abstract interface shared by all path-tracking controllers.
//
// Every algorithm (Pure Pursuit, Stanley, MPC, MPPI) implements this same
// interface, so the simulator and the benchmark harness can drive them
// interchangeably and compare their behaviour fairly.
//
#include "types.hpp"
#include <string>

namespace path_tracking {

class Controller {
public:
    virtual ~Controller() = default;

    // Human-readable algorithm name (used in CSV/report output).
    [[nodiscard]] virtual std::string name() const = 0;

    // Reset all internal state (call before each run).
    virtual void reset() = 0;

    // Compute the control command for one cycle.
    //   path  : the reference path
    //   state : current vehicle state
    //   dt    : control cycle time [s]
    [[nodiscard]] virtual ControlCommand
    compute(const Path& path, const VehicleState& state, double dt) = 0;
};

// ---------------------------------------------------------------------------
// Shared longitudinal speed tracking.
//
// All four lateral controllers share the same simple longitudinal law so that
// the comparison isolates *steering* behaviour. The target speed is the
// reference speed of the nearest path point; a proportional law produces the
// acceleration command, saturated to comfortable limits.
// ---------------------------------------------------------------------------
struct LongitudinalConfig {
    double kp{1.2};             // speed-error proportional gain [1/s]
    double accel_max{1.5};      // max acceleration [m/s^2]
    double decel_max{3.0};      // max deceleration [m/s^2] (magnitude)
};

[[nodiscard]] inline double
longitudinal_accel(double target_v, double current_v,
                   const LongitudinalConfig& cfg) {
    const double a = cfg.kp * (target_v - current_v);
    return clampd(a, -cfg.decel_max, cfg.accel_max);
}

} // namespace path_tracking
