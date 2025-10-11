#pragma once
#include <cmath>

struct State {
    double x{0.0}, y{0.0}, theta{0.0}, v{0.0};
};

struct Control {
    double steer{0.0}; // 前轮转角
    double a{0.0};     // 加速度
};

struct RobotSpec {
    double radius{0.3};
    double a_max{5.0};
    double steer_max{0.5};
    double L{1.0};
};

class BicycleModel {
public:
    BicycleModel(double dt, const RobotSpec& spec): dt_(dt), spec_(spec) {}

    State step(const State& s, const Control& u) const {
        State ns = s;
        double dtheta = (s.v / spec_.L) * std::tan(u.steer);
        ns.x += s.v * std::cos(s.theta) * dt_;
        ns.y += s.v * std::sin(s.theta) * dt_;
        ns.theta += dtheta * dt_;
        ns.v += u.a * dt_;
        return ns;
    }

    // 朝目标点的航向与速度调节（原始简化版）
    Control nominal(const State& s, double gx, double gy) const {
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

    // 基于参考切向与参考速度（不加入横向误差项）
    Control nominal_track_ref(const State& s, double desired_theta, double desired_v) const {
        Control u;
        double err_theta = wrapAngle(desired_theta - s.theta);
        u.steer = clamp(err_theta, -spec_.steer_max, spec_.steer_max);
        double err_v = desired_v - s.v;
        u.a = clamp(err_v, -spec_.a_max, spec_.a_max);
        return u;
    }

    const RobotSpec& spec() const { return spec_; }
    double dt() const { return dt_; }

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }
    static double wrapAngle(double a) {
        while (a > M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

private:
    double dt_;
    RobotSpec spec_;
};