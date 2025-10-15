#include "../include/bicycle_model.h"
#include <algorithm> // for std::max, std::min
#include <cmath>     // for M_PI, atan2, cos, sin, tan

namespace dpcbf_qp {

BicycleModel::BicycleModel(double dt, const RobotSpec& spec): dt_(dt), spec_(spec) {}

State BicycleModel::step(const State& s, const Control& u) const {
    State ns = s;
    double dtheta = (s.v / spec_.L) * std::tan(u.steer);
    ns.x += s.v * std::cos(s.theta) * dt_;
    ns.y += s.v * std::sin(s.theta) * dt_;
    ns.theta += dtheta * dt_;
    ns.v += u.a * dt_;
    return ns;
}

Control BicycleModel::nominal(const State& s, double gx, double gy) const {
    Control u;
    double desired_theta = std::atan2(gy - s.y, gx - s.x);
    double err_theta = wrapAngle(desired_theta - s.theta);
    u.steer = clamp(err_theta, -spec_.steer_max, spec_.steer_max);
    double desired_v = 1.0;
    double err_v = desired_v - s.v;
    // 弱化速度跟踪增益，避免加减速过强导致偏移
    u.a = clamp(0.3 * err_v, -spec_.a_max, spec_.a_max);
    return u;
}

Control BicycleModel::nominal_track_ref(const State& s, double desired_theta, double desired_v) const {
    Control u;
    double err_theta = wrapAngle(desired_theta - s.theta);
    u.steer = clamp(err_theta, -spec_.steer_max, spec_.steer_max);
    double err_v = desired_v - s.v;
    u.a = clamp(err_v, -spec_.a_max, spec_.a_max);
    return u;
}

double BicycleModel::clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

double BicycleModel::wrapAngle(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

} // namespace dpcbf_qp