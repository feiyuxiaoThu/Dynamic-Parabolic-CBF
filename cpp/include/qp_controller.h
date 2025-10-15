#pragma once
#include "bicycle_model.h"
#include "dpcbf.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <algorithm>
#include <cmath>

namespace dpcbf_qp {

// 纯 CBF-QP（单松弛 s）：
// min 0.5 (x - x_ref)^T W (x - x_ref)
// s.t. 变量盒约束 + 线性化的离散 DPCBF 约束
// x = [steer, a, s]，s为松弛变量(>=0)
struct QPWeights {
    double w_steer{12.0};
    double w_a{12.0};
    double w_jerk_steer{5.0};  // 转向角jerk惩罚权重
    double w_jerk_accel{200.0};  // 加速度jerk惩罚权重
    double rho{20.0}; // 松弛变量权重
};

struct DiscreteCBFConfig {
    double gamma{0.25};
    double du{1e-3};      // 数值梯度步长
    double s_max{10.0};   // 松弛变量上界
};

class QPController {
public:
    QPController(const BicycleModel& model,
                 const DPCBFParams& dparams,
                 const QPWeights& w,
                 const DiscreteCBFConfig& cfg);

    Control solve(const State& s, const Control& u_ref,
                  const std::vector<Obstacle>& obstacles) const;
    
    // 更新上一时刻控制输入（用于jerk计算）
    void updatePreviousControl(const Control& u_prev);

private:
    const BicycleModel& model_;
    DPCBFParams dparams_;
    QPWeights w_;
    DiscreteCBFConfig cfg_;
    
    // 存储上一时刻的控制输入，用于jerk惩罚
    mutable Control u_previous_{0.0, 0.0};
    mutable bool has_previous_{false};
};

} // namespace dpcbf_qp