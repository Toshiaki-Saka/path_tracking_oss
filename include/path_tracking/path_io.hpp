// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// path_io.hpp - Reference-path CSV loading and nearest-point search.
//
// Input CSV format (header row required):
//   x_m, y_m, yaw, curvature
// Columns are located by header name, so extra columns and any column order
// are accepted as long as those four are present. Arc length (s) is computed
// from the geometry; a reference speed profile is derived from curvature.
//
#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace path_tracking {

// Parameters for deriving a reference speed profile from path curvature.
struct SpeedProfileConfig {
    double v_max{8.0};          // straight-line cruise speed [m/s] (~29 km/h)
    double v_min{1.5};          // minimum speed in tight corners [m/s]
    double lat_accel_max{1.5};  // lateral-acceleration budget [m/s^2]
};

// Build a curvature-limited reference speed for each point:
//   v = clamp( sqrt(lat_accel_max / |kappa|), v_min, v_max )
inline void assign_speed_profile(Path& path, const SpeedProfileConfig& cfg) {
    for (auto& p : path.points()) {
        const double k = std::abs(p.curvature);
        double v = cfg.v_max;
        if (k > 1e-6) {
            v = std::sqrt(cfg.lat_accel_max / k);
        }
        p.v_ref = clampd(v, cfg.v_min, cfg.v_max);
    }
    // Smooth the profile so the vehicle decelerates *before* a corner.
    // Backward pass limiting deceleration to lat_accel_max-equivalent.
    const std::size_t n = path.size();
    for (std::size_t i = n; i-- > 1;) {
        const double ds = path[i].s - path[i - 1].s;
        if (ds <= 0.0) continue;
        const double v_allow =
            std::sqrt(path[i].v_ref * path[i].v_ref + 2.0 * cfg.lat_accel_max * ds);
        path[i - 1].v_ref = std::min(path[i - 1].v_ref, v_allow);
    }
}

namespace detail {

inline constexpr std::size_t npos = static_cast<std::size_t>(-1);

// Strip surrounding whitespace and quotes from a CSV header cell.
inline std::string trim(const std::string& s) {
    const auto is_pad = [](char c) {
        return c == ' ' || c == '\t' || c == '"' || c == '\'';
    };
    std::size_t b = 0, e = s.size();
    while (b < e && is_pad(s[b])) ++b;
    while (e > b && is_pad(s[e - 1])) --e;
    return s.substr(b, e - b);
}

}  // namespace detail

// Load a reference path from a CSV file. Throws std::runtime_error on failure.
inline Path load_path_csv(const std::string& filename,
                          const SpeedProfileConfig& speed_cfg = {}) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("cannot open path CSV: " + filename);
    }

    std::string line;
    // Header line - also strips a UTF-8 BOM if present.
    if (!std::getline(f, line)) {
        throw std::runtime_error("empty path CSV: " + filename);
    }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
        line.erase(0, 3);
    }

    // Locate the four required columns by header name.
    std::size_t idx_x = detail::npos, idx_y = detail::npos,
                idx_yaw = detail::npos, idx_k = detail::npos;
    {
        std::istringstream hs(line);
        std::string name;
        for (std::size_t i = 0; std::getline(hs, name, ','); ++i) {
            const std::string key = detail::trim(name);
            if (key == "x_m")            idx_x   = i;
            else if (key == "y_m")       idx_y   = i;
            else if (key == "yaw")       idx_yaw = i;
            else if (key == "curvature") idx_k   = i;
        }
    }
    if (idx_x == detail::npos || idx_y == detail::npos ||
        idx_yaw == detail::npos || idx_k == detail::npos) {
        throw std::runtime_error(
            "path CSV must have x_m, y_m, yaw, curvature columns: " + filename);
    }
    const std::size_t n_needed =
        std::max(std::max(idx_x, idx_y), std::max(idx_yaw, idx_k)) + 1;

    std::vector<PathPoint> pts;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        std::istringstream ss(line);
        std::string tok;
        std::vector<double> cols;
        cols.reserve(n_needed);
        while (cols.size() < n_needed && std::getline(ss, tok, ',')) {
            double v = 0.0;
            try { v = std::stod(tok); }
            catch (...) { v = 0.0; }
            cols.push_back(v);
        }
        if (cols.size() < n_needed) continue;

        PathPoint p;
        p.x         = cols[idx_x];
        p.y         = cols[idx_y];
        p.yaw       = cols[idx_yaw];
        p.curvature = cols[idx_k];
        pts.push_back(p);
    }
    if (pts.size() < 2) {
        throw std::runtime_error("path CSV has too few points: " + filename);
    }

    // Compute cumulative arc length.
    pts[0].s = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double dx = pts[i].x - pts[i - 1].x;
        const double dy = pts[i].y - pts[i - 1].y;
        pts[i].s = pts[i - 1].s + std::hypot(dx, dy);
    }

    Path path(std::move(pts));
    assign_speed_profile(path, speed_cfg);
    return path;
}

// Find the index of the path point nearest to (x, y).
// Linear scan - simple and robust for sample-sized routes.
[[nodiscard]] inline std::size_t nearest_index(const Path& path,
                                               double x, double y,
                                               std::size_t hint = 0,
                                               std::size_t window = 0) {
    const std::size_t n = path.size();
    std::size_t lo = 0, hi = n;
    if (window > 0 && hint < n) {
        lo = (hint > window) ? hint - window : 0;
        hi = std::min(n, hint + window);
    }
    std::size_t best = lo;
    double best_d2 = std::numeric_limits<double>::max();
    for (std::size_t i = lo; i < hi; ++i) {
        const double dx = path[i].x - x;
        const double dy = path[i].y - y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
}

// Signed cross-track error of the vehicle relative to the path point `idx`.
// Positive => vehicle is to the LEFT of the path tangent.
[[nodiscard]] inline double cross_track_error(const Path& path,
                                              std::size_t idx,
                                              double x, double y) {
    const PathPoint& p = path[idx];
    const double dx = x - p.x;
    const double dy = y - p.y;
    // Left-normal of the tangent is (-sin(yaw), cos(yaw)).
    return -std::sin(p.yaw) * dx + std::cos(p.yaw) * dy;
}

} // namespace path_tracking
