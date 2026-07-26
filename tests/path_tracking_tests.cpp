// SPDX-License-Identifier: Apache-2.0
//
// path_tracking_tests.cpp - Minimal regression tests for the header-only
// path-tracking library. No external test framework: a tiny assert harness
// keeps the dependency footprint identical to the library (std + Eigen).
//
// Coverage:
//   (a) cross_track_error sign and magnitude on known geometry
//   (b) nearest_index windowed search agrees with the full scan
//   (c) assign_speed_profile lowers v_ref as curvature increases
//   (d) Pure Pursuit converges to a straight line from a lateral offset
//
// Exits non-zero on any failure so `ctest` turns a regression into red.

#include "path_tracking/types.hpp"
#include "path_tracking/path_io.hpp"
#include "path_tracking/vehicle.hpp"
#include "path_tracking/pure_pursuit.hpp"
#include "path_tracking/simulator.hpp"

#include <cmath>
#include <cstdio>

using namespace path_tracking;

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}

void close(double value, double expected, double tol, const char* what) {
    const bool ok = std::fabs(value - expected) <= tol;
    if (ok) {
        std::printf("  [PASS] %s (%.6f ~ %.6f)\n", what, value, expected);
    } else {
        std::printf("  [FAIL] %s: got %.6f, expected %.6f +/- %.1e\n",
                    what, value, expected, tol);
        ++g_failures;
    }
}

// Build a straight reference path of `n` points spaced `ds` along the +x axis
// (yaw = 0, curvature = 0), with arc length and speed profile filled in.
Path straight_path(std::size_t n, double ds) {
    std::vector<PathPoint> pts(n);
    for (std::size_t i = 0; i < n; ++i) {
        pts[i].x = static_cast<double>(i) * ds;
        pts[i].y = 0.0;
        pts[i].yaw = 0.0;
        pts[i].curvature = 0.0;
        pts[i].s = static_cast<double>(i) * ds;
    }
    Path path(std::move(pts));
    assign_speed_profile(path, {});
    return path;
}

}  // namespace

int main() {
    std::printf("=== path_tracking regression tests ===\n");

    // (a) cross_track_error -------------------------------------------------
    std::printf("\n[a] cross_track_error sign / magnitude\n");
    {
        const Path xp = straight_path(10, 1.0);  // along +x, yaw 0
        // Left-normal of a +x tangent is +y, so a vehicle at +y is "left" (+).
        close(cross_track_error(xp, 5, xp[5].x, 1.5), +1.5, 1e-9,
              "left offset is positive");
        close(cross_track_error(xp, 5, xp[5].x, -2.0), -2.0, 1e-9,
              "right offset is negative");
        close(cross_track_error(xp, 5, xp[5].x, 0.0), 0.0, 1e-9,
              "on-path is zero");

        // Path along +y (yaw = pi/2): left-normal is -x, so a vehicle at -x
        // relative to the point is "left" (+).
        std::vector<PathPoint> yv(5);
        for (std::size_t i = 0; i < yv.size(); ++i) {
            yv[i].x = 0.0; yv[i].y = static_cast<double>(i);
            yv[i].yaw = kPi / 2.0; yv[i].s = static_cast<double>(i);
        }
        const Path yp(std::move(yv));
        close(cross_track_error(yp, 2, -1.0, yp[2].y), +1.0, 1e-9,
              "+y path: -x offset is positive");
    }

    // (b) nearest_index windowed vs full scan -------------------------------
    std::printf("\n[b] nearest_index windowed == full scan\n");
    {
        const Path p = straight_path(60, 1.0);
        // Query a point closest to index 37.
        const double qx = 37.2, qy = 0.3;
        const std::size_t full = nearest_index(p, qx, qy, 0, 0);
        const std::size_t win  = nearest_index(p, qx, qy, /*hint=*/35, /*window=*/10);
        check(full == 37, "full scan finds nearest index 37");
        check(win == full, "windowed search matches full scan");
        // Exact path point maps to itself.
        check(nearest_index(p, p[12].x, p[12].y, 0, 0) == 12,
              "exact point maps to its own index");
    }

    // (c) assign_speed_profile vs curvature ---------------------------------
    std::printf("\n[c] assign_speed_profile lowers v_ref with curvature\n");
    {
        const SpeedProfileConfig cfg;  // v_max 8, v_min 1.5, lat_accel 1.5
        std::vector<PathPoint> v(40);
        for (std::size_t i = 0; i < v.size(); ++i) {
            v[i].x = static_cast<double>(i);
            v[i].y = 0.0;
            v[i].s = static_cast<double>(i);
            // curvature strictly increasing from 0.
            v[i].curvature = 0.02 * static_cast<double>(i);
        }
        Path p(std::move(v));
        assign_speed_profile(p, cfg);

        bool non_increasing = true, in_bounds = true;
        for (std::size_t i = 0; i + 1 < p.size(); ++i) {
            if (p[i].v_ref < p[i + 1].v_ref - 1e-9) non_increasing = false;
        }
        for (std::size_t i = 0; i < p.size(); ++i) {
            if (p[i].v_ref < cfg.v_min - 1e-9 || p[i].v_ref > cfg.v_max + 1e-9)
                in_bounds = false;
        }
        check(non_increasing, "v_ref is non-increasing as curvature grows");
        check(in_bounds, "v_ref stays within [v_min, v_max]");
        check(p[0].v_ref > p[p.size() - 1].v_ref,
              "straightest point is faster than the tightest");
    }

    // (d) Pure Pursuit converges to a straight line -------------------------
    std::printf("\n[d] Pure Pursuit converges from a lateral offset\n");
    {
        const Path path = straight_path(600, 0.5);  // 300 m straight
        const double dt = 0.01;
        Vehicle veh(2.7, dt);
        VehicleState s0;
        s0.x = 0.0; s0.y = 1.0; s0.yaw = 0.0;        // start 1 m to the left
        s0.v = path[0].v_ref * 0.5;
        veh.set_state(s0);

        PurePursuitController ctrl{};
        ctrl.reset();

        const double cte0 = std::fabs(cross_track_error(path, 0, s0.x, s0.y));
        double late_max = 0.0;  // worst |cte| in the settled phase
        std::size_t hint = 0;
        const int steps = 1500;  // ~ vehicle travels < 300 m, stays on path
        for (int k = 0; k < steps; ++k) {
            const VehicleState& st = veh.state();
            const ControlCommand cmd = ctrl.compute(path, st, dt);
            hint = nearest_index(path, st.x, st.y, hint, 400);
            const double cte = std::fabs(cross_track_error(path, hint, st.x, st.y));
            if (k > 1000) late_max = std::max(late_max, cte);
            veh.step(cmd.accel, cmd.steer);
        }

        check(cte0 > 0.9, "starts with ~1 m lateral error");
        check(late_max < 0.1,
              "settled cross-track error < 0.1 m after convergence");
    }

    // Verdict ---------------------------------------------------------------
    std::printf("\n%s (%d failure(s))\n",
                g_failures ? "TESTS FAILED" : "ALL TESTS PASSED", g_failures);
    return g_failures ? 1 : 0;
}
