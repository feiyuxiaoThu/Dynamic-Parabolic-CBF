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
    BicycleModel(double dt, const RobotSpec& spec);

    State step(const State& s, const Control& u) const;
    Control nominal(const State& s, double gx, double gy) const;
    Control nominal_track_ref(const State& s, double desired_theta, double desired_v) const;

    const RobotSpec& spec() const { return spec_; }
    double dt() const { return dt_; }

    static double clamp(double x, double lo, double hi);
    static double wrapAngle(double a);

private:
    double dt_;
    RobotSpec spec_;
};