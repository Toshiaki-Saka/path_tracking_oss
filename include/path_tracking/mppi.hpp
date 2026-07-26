// SPDX-License-Identifier: Apache-2.0
#pragma once
//
// mppi.hpp - Model Predictive Path Integral path-tracking controller.
//
// Sampling-based predictive controller (2016, Williams et al.). Many noisy
// steering sequences are rolled out through the kinematic model; each is
// scored by a cost; the next control is the cost-weighted average:
//
//   u* = sum_k w(tau_k) * u_k ,   w(tau_k) proportional to exp(-S_k / lambda)
//
//   + Works with non-convex / non-differentiable costs; naturally parallel;
//     robust in dynamic environments.
//   - Quality depends on the number of samples; weaker theoretical
//     guarantees; needs hyper-parameter tuning (noise sigma, temperature).
//
// This implementation uses a deterministic seed so benchmark runs are
// reproducible.
//
#include "controller.hpp"
#include "path_io.hpp"
#include <cmath>
#include <random>
#include <vector>

namespace path_tracking {

struct MppiConfig {
    double wheelbase{2.7};      // [m]
    int    horizon{20};         // prediction steps N
    int    samples{120};        // number of rollouts K
    double pred_dt{0.05};       // prediction step [s]
    double noise_sigma{0.08};   // steering exploration noise std-dev [rad]
    double lambda{1.0};         // temperature (lower = greedier)
    double w_cte{8.0};          // cross-track error weight
    double w_yaw{3.0};          // heading error weight
    double w_steer{0.3};        // steering magnitude weight
    double steer_limit{0.6};    // |steer| limit [rad]
    double steer_rate_limit{4.0}; // |d(steer)/dt| limit [rad/s]
    unsigned seed{12345u};      // RNG seed (fixed for reproducibility)
    LongitudinalConfig lon{};
};

class MppiController final : public Controller {
public:
    explicit MppiController(MppiConfig cfg = {})
        : cfg_(cfg),
          nominal_(static_cast<std::size_t>(cfg.horizon), 0.0),
          rng_(cfg.seed) {}

    [[nodiscard]] std::string name() const override { return "MPPI"; }

    void reset() override {
        last_idx_ = 0;
        prev_steer_ = 0.0;
        std::fill(nominal_.begin(), nominal_.end(), 0.0);
        rng_.seed(cfg_.seed);
    }

    [[nodiscard]] ControlCommand
    compute(const Path& path, const VehicleState& s, double dt) override {
        ControlCommand cmd{};
        const std::size_t N = nominal_.size();
        const std::size_t K = static_cast<std::size_t>(cfg_.samples);

        const std::size_t near = nearest_index(path, s.x, s.y, last_idx_, 300);
        last_idx_ = near;

        // Warm-start: shift the nominal sequence one step forward.
        for (std::size_t k = 0; k + 1 < N; ++k) nominal_[k] = nominal_[k + 1];

        std::normal_distribution<double> noise(0.0, cfg_.noise_sigma);

        std::vector<std::vector<double>> perturb(K, std::vector<double>(N, 0.0));
        std::vector<double> costs(K, 0.0);
        double min_cost = std::numeric_limits<double>::max();

        // Sample K perturbed rollouts and score each.
        for (std::size_t k = 0; k < K; ++k) {
            std::vector<double> seq(N);
            for (std::size_t i = 0; i < N; ++i) {
                perturb[k][i] = noise(rng_);
                seq[i] = clampd(nominal_[i] + perturb[k][i],
                                -cfg_.steer_limit, cfg_.steer_limit);
            }
            costs[k] = rollout_cost(path, s, seq);
            min_cost = std::min(min_cost, costs[k]);
        }

        // Cost-weighted average of the perturbations (path-integral update).
        std::vector<double> weight(K, 0.0);
        double w_sum = 0.0;
        for (std::size_t k = 0; k < K; ++k) {
            weight[k] = std::exp(-(costs[k] - min_cost) / cfg_.lambda);
            w_sum += weight[k];
        }
        if (w_sum < 1e-12) w_sum = 1.0;

        for (std::size_t i = 0; i < N; ++i) {
            double upd = 0.0;
            for (std::size_t k = 0; k < K; ++k) {
                upd += weight[k] * perturb[k][i];
            }
            nominal_[i] = clampd(nominal_[i] + upd / w_sum,
                                 -cfg_.steer_limit, cfg_.steer_limit);
        }

        double steer = nominal_[0];

        // Steering-rate limit for smoother, more realistic actuation.
        const double max_step = cfg_.steer_rate_limit * dt;
        steer = clampd(steer, prev_steer_ - max_step, prev_steer_ + max_step);
        prev_steer_ = steer;

        cmd.steer = steer;
        cmd.accel = longitudinal_accel(path[near].v_ref, s.v, cfg_.lon);
        return cmd;
    }

private:
    [[nodiscard]] double rollout_cost(const Path& path,
                                      const VehicleState& s0,
                                      const std::vector<double>& seq) const {
        VehicleState st = s0;
        double cost = 0.0;
        std::size_t hint = last_idx_;

        for (double steer : seq) {
            st.x  += st.v * std::cos(st.yaw) * cfg_.pred_dt;
            st.y  += st.v * std::sin(st.yaw) * cfg_.pred_dt;
            st.yaw = wrap_to_pi(st.yaw + st.v * std::tan(steer)
                                / cfg_.wheelbase * cfg_.pred_dt);

            hint = nearest_index(path, st.x, st.y, hint, 200);
            const double cte = cross_track_error(path, hint, st.x, st.y);
            const double yaw_err = wrap_to_pi(path[hint].yaw - st.yaw);

            cost += cfg_.w_cte   * cte * cte
                  + cfg_.w_yaw   * yaw_err * yaw_err
                  + cfg_.w_steer * steer * steer;
        }
        return cost;
    }

    MppiConfig          cfg_;
    std::vector<double> nominal_;
    std::mt19937        rng_;
    std::size_t         last_idx_{0};
    double              prev_steer_{0.0};
};

} // namespace path_tracking
