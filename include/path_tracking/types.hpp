// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// types.hpp - Common geometric types for the path-tracking library.
//
// This is an open-source vehicle path-tracking sample. All quantities use
// SI units (metres, radians, m/s) and IEEE-754 double precision so the code
// is portable and easy to read; no fixed-point or vendor-specific encoding.
//
#include <cmath>
#include <cstddef>
#include <vector>

namespace path_tracking {

inline constexpr double kPi = 3.14159265358979323846;

// Wrap an angle into (-pi, pi].
[[nodiscard]] inline double wrap_to_pi(double a) noexcept {
    while (a >  kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

[[nodiscard]] inline double clampd(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// Vehicle pose / state.
// ---------------------------------------------------------------------------
struct VehicleState {
    double x{0.0};      // [m]
    double y{0.0};      // [m]
    double yaw{0.0};    // [rad], wrapped to (-pi, pi]
    double v{0.0};      // [m/s]
};

// ---------------------------------------------------------------------------
// A single reference-path point.
//   x, y       : position [m]
//   yaw        : path tangent heading [rad]
//   curvature  : signed path curvature [1/m]
//   s          : arc length from path start [m]
//   v_ref      : reference speed at this point [m/s]
// ---------------------------------------------------------------------------
struct PathPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double curvature{0.0};
    double s{0.0};
    double v_ref{0.0};
};

// A reference path is just a vector of points plus convenience accessors.
class Path {
public:
    Path() = default;
    explicit Path(std::vector<PathPoint> pts) : pts_(std::move(pts)) {}

    [[nodiscard]] std::size_t size()  const noexcept { return pts_.size(); }
    [[nodiscard]] bool        empty() const noexcept { return pts_.empty(); }

    [[nodiscard]] const PathPoint& operator[](std::size_t i) const { return pts_[i]; }
    [[nodiscard]] PathPoint&       operator[](std::size_t i)       { return pts_[i]; }

    [[nodiscard]] const std::vector<PathPoint>& points() const noexcept { return pts_; }
    [[nodiscard]] std::vector<PathPoint>&       points()       noexcept { return pts_; }

    [[nodiscard]] double total_length() const noexcept {
        return pts_.empty() ? 0.0 : pts_.back().s;
    }

private:
    std::vector<PathPoint> pts_;
};

// ---------------------------------------------------------------------------
// Control command produced by a path-tracking controller for one cycle.
//   steer : front-wheel steering angle [rad] (+: turn left / CCW)
//   accel : longitudinal acceleration  [m/s^2]
// ---------------------------------------------------------------------------
struct ControlCommand {
    double steer{0.0};
    double accel{0.0};
};

} // namespace path_tracking
