#pragma once
#include "bicycle_model.hpp"
#include "dpcbf.hpp"
#include <vector>

// 迭代修正器：近似解决 "min ||u-u_ref||^2 s.t. h_{k+1}-h_k + gamma h_k >= 0"
// 通过数值线性化对每个障碍构造 d_h 对 u 的梯度，沿着约束违反方向推 u
struct CBFControllerConfig {
    double gamma{0.5};       // 离散稳定项
    double step_gain{0.8};   // 修正步长
    int max_iters{10};
};

class CBFController {
public:
    CBFController(const BicycleModel& model, const DPCBFParams& params, const CBFControllerConfig& cfg)
    : model_(model), params_(params), cfg_(cfg) {}

    Control solve(const State& s, const Control& u_ref,
                  const std::vector<Obstacle>& obstacles) const {
        Control u = u_ref;
        // 迭代修正
        for (int it=0; it<cfg_.max_iters; ++it) {
            bool all_ok = true;
            for (const auto& obs : obstacles) {
                // 评估当前约束
                State s1 = model_.step(s, u);
                auto hd = dpcbf_discrete(s.x, s.y, s.theta, s.v,
                                         s1.x, s1.y, s1.theta, s1.v,
                                         obs, model_.spec().radius, params_);
                double c = hd[1] + cfg_.gamma * hd[0]; // 约束：c >= 0
                if (c < 0.0) {
                    all_ok = false;
                    // 数值梯度 dc/du ≈ [dc/d(steer), dc/da]
                    Control grad{};
                    const double du = 1e-3;
                    // steer
                    {
                        Control u2 = u; u2.steer += du;
                        State s2 = model_.step(s, u2);
                        auto hd2 = dpcbf_discrete(s.x, s.y, s.theta, s.v,
                                                  s2.x, s2.y, s2.theta, s2.v,
                                                  obs, model_.spec().radius, params_);
                        double c2 = hd2[1] + cfg_.gamma * hd2[0];
                        grad.steer = (c2 - c) / du;
                    }
                    // a
                    {
                        Control u2 = u; u2.a += du;
                        State s2 = model_.step(s, u2);
                        auto hd2 = dpcbf_discrete(s.x, s.y, s.theta, s.v,
                                                  s2.x, s2.y, s2.theta, s2.v,
                                                  obs, model_.spec().radius, params_);
                        double c2 = hd2[1] + cfg_.gamma * hd2[0];
                        grad.a = (c2 - c) / du;
                    }
                    // 沿梯度上升以增大 c
                    double gnorm = std::sqrt(grad.steer*grad.steer + grad.a*grad.a) + 1e-9;
                    Control delta;
                    delta.steer = cfg_.step_gain * (-c) * (grad.steer / gnorm);
                    delta.a     = cfg_.step_gain * (-c) * (grad.a     / gnorm);

                    u.steer = BicycleModel::clamp(u.steer + delta.steer, -model_.spec().steer_max, model_.spec().steer_max);
                    u.a     = BicycleModel::clamp(u.a     + delta.a,     -model_.spec().a_max,      model_.spec().a_max);
                }
            }
            if (all_ok) break;
        }
        return u;
    }

private:
    const BicycleModel& model_;
    DPCBFParams params_;
    CBFControllerConfig cfg_;
};