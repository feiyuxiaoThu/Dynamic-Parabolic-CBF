#pragma once
#include <cmath>
#include <array>

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

inline double norm2(double x, double y) {
    return std::sqrt(x*x + y*y);
}

// 将 v_rel 旋转到 LoS 坐标：x 轴对准 p_rel
inline void rotateToLOS(double px, double py, double vx, double vy, double& vxl, double& vyl) {
    double rot = std::atan2(py, px);
    // R = [[cos, sin],[-sin, cos]]，v_l = R v
    double c = std::cos(rot), s = std::sin(rot);
    vxl = c*vx + s*vy;
    vyl = -s*vx + c*vy;
}

// 连续时间 DPCBF h(x)
inline DPCBFResult dpcbf_continuous(double X, double Y, double theta, double v,
                                    const Obstacle& obs, double robot_radius,
                                    const DPCBFParams& p) {
    // 相对量
    double px = obs.ox - X;
    double py = obs.oy - Y;
    double vx_rel = obs.vx - v*std::cos(theta);
    double vy_rel = obs.vy - v*std::sin(theta);

    double pr = norm2(px, py);
    double vr = norm2(vx_rel, vy_rel);

    double ego_dim = (obs.r + robot_radius) * p.margin;
    double dsafe = std::max(pr*pr - ego_dim*ego_dim, p.eps);

    // LoS 旋转
    double vxl=0.0, vyl=0.0;
    rotateToLOS(px, py, vx_rel, vy_rel, vxl, vyl);

    // λ(x), μ(x)
    // 依据论文与离散实现，对 k_lambda, k_mu 进行尺度缩放：k_*' = k_* * sqrt(margin^2 - 1) / ego_dim
    double scale = std::sqrt(p.margin*p.margin - 1.0) / (ego_dim + p.eps);
    double lam = (p.k_lambda * scale) * std::sqrt(dsafe) / (vr + p.eps);
    double mu  = (p.k_mu     * scale) * std::sqrt(dsafe);

    DPCBFResult res;
    res.h = vxl + lam * (vyl*vyl) + mu;
    return res;
}

// 离散版：返回 h_k 与 d_h = h_{k+1} - h_k
inline std::array<double,2> dpcbf_discrete(double X, double Y, double theta, double v,
                                            double X1, double Y1, double theta1, double v1,
                                            const Obstacle& obs, double robot_radius,
                                            const DPCBFParams& p) {
    auto hk = dpcbf_continuous(X, Y, theta, v, obs, robot_radius, p).h;
    auto h1 = dpcbf_continuous(X1, Y1, theta1, v1, obs, robot_radius, p).h;
    return {hk, h1 - hk};
}