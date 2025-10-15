#pragma once
#include <cmath>
#include <array>

namespace dpcbf_qp {

struct Obstacle {
    double ox{0.0}, oy{0.0}, r{0.0}, vx{0.0}, vy{0.0};
};

struct DPCBFParams {
    double k_lambda{0.1};
    double k_mu{0.5};
    double margin{1.05}; // 安全系数 s / beta
    double eps{1e-6};
};

struct DPCBFResult {
    double h{0.0};
};

// 连续时间 DPCBF h(x)
DPCBFResult dpcbf_continuous(double X, double Y, double theta, double v,
                                    const Obstacle& obs, double robot_radius,
                                    const DPCBFParams& p);

// 离散版：返回 h_k 与 d_h = h_{k+1} - h_k
std::array<double,2> dpcbf_discrete(double X, double Y, double theta, double v,
                                            double X1, double Y1, double theta1, double v1,
                                            const Obstacle& obs, double robot_radius,
                                            const DPCBFParams& p);

} // namespace dpcbf_qp