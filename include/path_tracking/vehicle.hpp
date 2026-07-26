// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// vehicle.hpp - Kinematic bicycle model.
//
// State: x [m], y [m], yaw [rad] (wrapped to (-pi, pi]), v [m/s].
// One control cycle integrates with explicit Euler at a fixed time step.
// No mass / tyre / inertia dynamics are modelled - this is a kinematic
// model suitable for path-tracking algorithm comparison.
//
#include "types.hpp"
#include <algorithm>
#include <cmath>

namespace path_tracking {

class Vehicle {
public:
    // wheelbase : axle-to-axle distance [m]
    // dt        : integration step [s]
    explicit Vehicle(double wheelbase = 2.7, double dt = 0.01) noexcept
        : wheelbase_(wheelbase), dt_(dt) {}

    void set_state(const VehicleState& s) noexcept { state_ = s; }
    [[nodiscard]] const VehicleState& state() const noexcept { return state_; }

    [[nodiscard]] double wheelbase() const noexcept { return wheelbase_; }
    [[nodiscard]] double dt()        const noexcept { return dt_; }

    // Integrate one control cycle.
    //   accel       : longitudinal acceleration [m/s^2]
    //   steer_angle : front-wheel steering angle [rad] (+: turn left)
    void step(double accel, double steer_angle) noexcept {
        VehicleState& s = state_;
        s.v   = std::max(0.0, s.v + accel * dt_);
        s.x  += s.v * std::cos(s.yaw) * dt_;
        s.y  += s.v * std::sin(s.yaw) * dt_;
        s.yaw = wrap_to_pi(s.yaw + s.v * std::tan(steer_angle) / wheelbase_ * dt_);
    }

private:
    VehicleState state_{};
    double       wheelbase_{2.7};
    double       dt_{0.01};
};

} // namespace path_tracking
