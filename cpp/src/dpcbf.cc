#include "../include/dpcbf.h"
#include <algorithm> // for std::max
#include <cmath>     // for sqrt, atan2, cos, sin

namespace {
    double norm2(double x, double y) {
        return std::sqrt(x*x + y*y);
    }

    void rotateToLOS(double px, double py, double vx, double vy, double& vxl, double& vyl) {
        double rot = std::atan2(py, px);
        double c = std::cos(rot), s = std::sin(rot);
        vxl = c*vx + s*vy;
        vyl = -s*vx + c*vy;
    }
} // namespace

namespace dpcbf_qp {

DPCBFResult dpcbf_continuous(double X, double Y, double theta, double v,
                                    const Obstacle& obs, double robot_radius,
                                    const DPCBFParams& p) {
    double px = obs.ox - X;
    double py = obs.oy - Y;
    double vx_rel = obs.vx - v*std::cos(theta);
    double vy_rel = obs.vy - v*std::sin(theta);

    double pr = norm2(px, py);
    double vr = norm2(vx_rel, vy_rel);

    double ego_dim = (obs.r + robot_radius) * p.margin;
    double dsafe = std::max(pr*pr - ego_dim*ego_dim, p.eps);

    double vxl=0.0, vyl=0.0;
    rotateToLOS(px, py, vx_rel, vy_rel, vxl, vyl);

    double scale = std::sqrt(p.margin*p.margin - 1.0) / (ego_dim + p.eps);
    double lam = (p.k_lambda * scale) * std::sqrt(dsafe) / (vr + p.eps);
    double mu  = (p.k_mu     * scale) * std::sqrt(dsafe);

    DPCBFResult res;
    res.h = vxl + lam * (vyl*vyl) + mu;
    return res;
}

std::array<double,2> dpcbf_discrete(double X, double Y, double theta, double v,
                                            double X1, double Y1, double theta1, double v1,
                                            const Obstacle& obs, double robot_radius,
                                            const DPCBFParams& p) {
    auto hk = dpcbf_continuous(X, Y, theta, v, obs, robot_radius, p).h;
    auto h1 = dpcbf_continuous(X1, Y1, theta1, v1, obs, robot_radius, p).h;
    return {hk, h1 - hk};
}

} // namespace dpcbf_qp