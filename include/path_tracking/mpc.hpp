// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// mpc.hpp - Model Predictive Control path-tracking controller (Eigen-based).
//
// Linear time-varying MPC on the lateral error dynamics. The tracking error
// is expressed as a 4-state model around the reference path:
//
//   state  z = [ e_cte, d(e_cte)/dt, e_yaw, d(e_yaw)/dt ]^T
//   input  u = steering angle [rad]
//
//   z_{k+1} = A z_k + B u_k
//
// The finite-horizon quadratic cost
//
//   J = sum_k ( z_k^T Q z_k + R u_k^2 )  +  z_N^T Qf z_N
//
// is minimised by the backward Riccati recursion, giving a time-varying
// feedback law u_k = -K_k z_k. Only the first input is applied; the horizon
// is recomputed every cycle (receding horizon).
//
//   + Explicit prediction horizon; jointly weighs lateral error, heading
//     error and control effort; solid theoretical foundation (LQR/Riccati).
//   - Higher compute cost than geometric trackers; relies on a linearised
//     model, so large model mismatch degrades accuracy.
//
// Uses Eigen for the matrix algebra (Eigen3, header-only).
//
#include "controller.hpp"
#include "path_io.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace path_tracking {

struct MpcConfig {
    double wheelbase{2.7};      // [m]
    int    horizon{30};         // prediction steps N
    double pred_dt{0.05};       // prediction step [s]
    double q_cte{12.0};         // cross-track error weight
    double q_cte_rate{2.0};     // cross-track error-rate weight
    double q_yaw{8.0};          // heading error weight
    double q_yaw_rate{2.0};     // heading error-rate weight
    double r_steer{12.0};       // steering effort weight (higher = smoother)
    double qf_scale{4.0};       // terminal-cost multiplier
    double steer_limit{0.6};    // |steer| limit [rad]
    double steer_rate_limit{4.0}; // |d(steer)/dt| limit [rad/s]
    LongitudinalConfig lon{};
};

class MpcController final : public Controller {
public:
    explicit MpcController(MpcConfig cfg = {}) : cfg_(cfg) {}

    [[nodiscard]] std::string name() const override { return "MPC"; }

    void reset() override { last_idx_ = 0; prev_steer_ = 0.0; }

    [[nodiscard]] ControlCommand
    compute(const Path& path, const VehicleState& s, double dt) override {
        using Eigen::Matrix4d;
        using Eigen::Vector4d;

        ControlCommand cmd{};
        const std::size_t near = nearest_index(path, s.x, s.y, last_idx_, 300);
        last_idx_ = near;

        const double v  = std::max(s.v, 0.5);   // avoid singular dynamics at v=0
        const double pdt = cfg_.pred_dt;
        const int    N  = cfg_.horizon;
        const double L  = cfg_.wheelbase;

        // ---- Current tracking-error state -----------------------------------
        // e_cte : signed cross-track error [m]   (+ : vehicle left of path)
        // e_yaw : heading error [rad]            (+ : vehicle yaw left of path)
        const double e_cte = cross_track_error(path, near, s.x, s.y);
        const double e_yaw = wrap_to_pi(s.yaw - path[near].yaw);

        Vector4d z;
        z << e_cte, v * std::sin(e_yaw), e_yaw, 0.0;

        // ---- Stage weights --------------------------------------------------
        Matrix4d Q = Matrix4d::Zero();
        Q(0, 0) = cfg_.q_cte;
        Q(1, 1) = cfg_.q_cte_rate;
        Q(2, 2) = cfg_.q_yaw;
        Q(3, 3) = cfg_.q_yaw_rate;
        const Matrix4d Qf = cfg_.qf_scale * Q;
        const double   R  = cfg_.r_steer;

        // ---- Linearised lateral error model (constant over the horizon) -----
        // Standard kinematic lateral-error dynamics, discretised (Euler):
        //   e_cte'      = e_cte + dt * e_cte_dot
        //   e_cte_dot'  = e_cte_dot + dt * v * e_yaw       (small-angle)
        //   e_yaw'      = e_yaw + dt * e_yaw_dot
        //   e_yaw_dot'  = (v/L) * steer
        Matrix4d A = Matrix4d::Identity();
        A(0, 1) = pdt;
        A(1, 2) = pdt * v;
        A(2, 3) = pdt;

        Vector4d B = Vector4d::Zero();
        B(3) = v / L;   // steering enters the yaw-rate channel

        // ---- Backward Riccati recursion -------------------------------------
        // P_N = Qf ;  P_k = Q + A^T P_{k+1} A
        //                    - A^T P_{k+1} B (R + B^T P_{k+1} B)^-1 B^T P_{k+1} A
        std::vector<Eigen::RowVector4d> K(static_cast<std::size_t>(N));
        Matrix4d P = Qf;
        for (int k = N - 1; k >= 0; --k) {
            const double denom = R + (B.transpose() * P * B)(0, 0);
            const Eigen::RowVector4d Kk = (B.transpose() * P * A) / denom;
            K[static_cast<std::size_t>(k)] = Kk;
            P = Q + A.transpose() * P * A - A.transpose() * P * B * Kk;
        }

        // ---- Apply only the first feedback gain (receding horizon) ----------
        const double steer_fb = -(K[0] * z)(0, 0);

        // ---- Curvature feed-forward -----------------------------------------
        // Steady-state steering needed to follow the local path curvature:
        //   steer_ff = atan(L * kappa)
        const double kappa    = path[near].curvature;
        const double steer_ff = std::atan(L * kappa);

        double steer = clampd(steer_ff + steer_fb,
                              -cfg_.steer_limit, cfg_.steer_limit);

        // Steering-rate limit for smoother, more realistic actuation.
        const double max_step = cfg_.steer_rate_limit * dt;
        steer = clampd(steer, prev_steer_ - max_step, prev_steer_ + max_step);
        prev_steer_ = steer;

        cmd.steer = steer;
        cmd.accel = longitudinal_accel(path[near].v_ref, s.v, cfg_.lon);
        return cmd;
    }

private:
    MpcConfig   cfg_;
    std::size_t last_idx_{0};
    double      prev_steer_{0.0};
};

} // namespace path_tracking
