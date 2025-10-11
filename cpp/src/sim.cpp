#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "../include/bicycle_model.hpp"
#include "../include/dpcbf.hpp"
#include "../include/qp_controller.hpp"
#include "../include/SplineTrajectory.hpp"

struct Waypoint { double x, y, theta; };

static double dist2d(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

int main(int argc, char** argv) {
    // 可自定义参考速度（命令行参数），默认 1.0 m/s
    double v_ref = 3.0;
    if (argc > 1) {
        try { v_ref = std::stod(argv[1]); } catch (...) { std::cerr << "Invalid v_ref arg, using default 1.0\n"; }
    }

    RobotSpec spec;
    spec.radius = 0.3;
    spec.a_max = 5.0;
    spec.steer_max = 0.5;
    spec.L = 1.0;
    double dt = 0.05;

    BicycleModel model(dt, spec);

    DPCBFParams dparams;
    dparams.k_lambda = 0.04;
    dparams.k_mu     = 0.18;
    dparams.margin   = 1.05;

    QPWeights w;
    w.w_steer = 12.0;
    w.w_a     = 5.0;
    w.rho     = 20.0;

    DiscreteCBFConfig cfg;
    cfg.gamma = 0.22;
    cfg.du    = 1e-3;
    cfg.s_max = 10.0;

    QPController controller(model, dparams, w, cfg);

    std::vector<Waypoint> wps = {{1.0, 7.5,0.0}, {30.0, 7.5,0.0}};

    // 使用 QuinticSpline3D 构造参考轨迹（2D，第三维为0），时间段由距离/参考速度分配
    Eigen::MatrixXd route(3, static_cast<int>(wps.size()));
    for (int i = 0; i < (int)wps.size(); ++i) {
        route(0, i) = wps[i].x;
        route(1, i) = wps[i].y;
        route(2, i) = 0.0;
    }
    Eigen::VectorXd ts(static_cast<int>(wps.size()) - 1);
    for (int i = 0; i < ts.size(); ++i) {
        double dx = route(0, i+1) - route(0, i);
        double dy = route(1, i+1) - route(1, i);
        double dist = std::sqrt(dx*dx + dy*dy);
        ts(i) = dist / std::max(1e-6, v_ref);
    }
    SplineTrajectory::SplineVector3D spline_points;
    spline_points.reserve(route.cols());
    for (int i = 0; i < route.cols(); ++i) {
        SplineTrajectory::SplinePoint3d p; p << route(0,i), route(1,i), route(2,i);
        spline_points.push_back(p);
    }
    std::vector<double> time_segments(ts.size());
    for (int i = 0; i < ts.size(); ++i) time_segments[i] = ts(i);
    SplineTrajectory::BoundaryConditions<3> boundary;
    boundary.start_velocity.setZero();
    boundary.start_acceleration.setZero();
    boundary.end_velocity.setZero();
    boundary.end_acceleration.setZero();
    SplineTrajectory::QuinticSpline3D quinticSpline;
    quinticSpline.update(time_segments, spline_points, 0.0, boundary);

    State s;
    s.x = wps[0].x;
    s.y = wps[0].y;
    s.theta = 0.0;
    s.v = 1.0;

    std::vector<Obstacle> obstacles = {
        {8.0,  9.0, 0.5, -0.5,  0.5},
        {10.0, 4.0, 0.5, -0.5, -0.5},
        {12.0, 5.0, 0.5, -0.5,  0.5},
        {14.0, 9.0, 0.5, -0.5, -0.5},
        {16.0, 6.0, 0.5, -0.5,  0.5},
        {18.0,14.0, 0.5, -0.5, -0.5},
        {20.0, 4.0, 0.5, -0.5,  0.5},
        {22.0,12.0, 0.5, -0.5, -0.5}
    };

    std::ofstream ofs("output_dpcbf.csv");
    ofs << "t,x,y,theta,v,steer,a,h_min,ref_x,ref_y";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        ofs << ",obs" << i << "_ox,obs" << i << "_oy,obs" << i << "_r,obs" << i << "_vx,obs" << i << "_vy";
    }
    ofs << "\n";

    double T = quinticSpline.getTrajectory().getDuration();
    int steps = static_cast<int>(T / dt);

    // 控制变化限幅参数（可调）
    Control u_prev; u_prev.steer = 0.0; u_prev.a = 0.0;
    const double dsteer_max = 0.05; // 每步最大转角变化 [rad/step]
    const double da_max_step = 0.5; // 每步最大加速度变化 [m/s^2/step]

    for (int k=0; k<steps; ++k) {
        double t = k * dt;

        for (auto& ob : obstacles) {
            ob.ox += ob.vx * dt;
            ob.oy += ob.vy * dt;
        }

        // 参考轨迹按时间评估（时间夹紧）
        double t_eval = std::min(t, quinticSpline.getTrajectory().getDuration());
        auto pos = quinticSpline.getTrajectory().getPos(t_eval);
        auto vel = quinticSpline.getTrajectory().getVel(t_eval);
        auto acc = quinticSpline.getTrajectory().getAcc(t_eval);
        double x_ref = pos(0);
        double y_ref = pos(1);
        double vx = vel(0), vy = vel(1);
        double ax = acc(0), ay = acc(1);
        double vel_norm = std::sqrt(vx*vx + vy*vy);
        double desired_theta = (vel_norm > 1e-6) ? std::atan2(vy, vx)
                                                 : std::atan2(y_ref - s.y, x_ref - s.x);

        // 曲率自适应速度规划：v_des <= sqrt(a_lat_max / |kappa|)
        double kappa_eps = 1e-6;
        double denom = std::pow(std::max(vel_norm, kappa_eps), 3);
        double kappa = (vel_norm > kappa_eps) ? ((vx*ay - vy*ax) / denom) : 0.0;
        double a_lat_max = 2.0; // 最大允许横向加速度，可调
        double v_cap = (std::abs(kappa) > kappa_eps) ? std::sqrt(a_lat_max / std::abs(kappa)) : v_ref;
        double desired_v = std::min(v_ref, v_cap);

        // 名义控制（简化版）
        Control u_ref = model.nominal_track_ref(s, desired_theta, desired_v);

        // CBF-QP（含参考跟踪代价）修正
        Control u_raw = controller.solve(s, u_ref, obstacles, x_ref, y_ref, desired_theta);

        // 施加控制变化限幅，避免快速打舵/猛加减速造成回旋
        double steer_cmd = BicycleModel::clamp(
            u_prev.steer + BicycleModel::clamp(u_raw.steer - u_prev.steer, -dsteer_max, dsteer_max),
            -spec.steer_max, spec.steer_max
        );
        double a_cmd = BicycleModel::clamp(
            u_prev.a + BicycleModel::clamp(u_raw.a - u_prev.a, -da_max_step, da_max_step),
            -spec.a_max, spec.a_max
        );
        Control u; u.steer = steer_cmd; u.a = a_cmd;

        State s1 = model.step(s, u);

        double hmin = 1e9;
        for (const auto& ob : obstacles) {
            auto hres = dpcbf_continuous(s.x, s.y, s.theta, s.v, ob, spec.radius, dparams);
            hmin = std::min(hmin, hres.h);
        }

        ofs << t << "," << s.x << "," << s.y << "," << s.theta << "," << s.v
            << "," << u.steer << "," << u.a << "," << hmin << "," << x_ref << "," << y_ref;
        for (const auto& ob : obstacles) {
            ofs << "," << ob.ox << "," << ob.oy << "," << ob.r << "," << ob.vx << "," << ob.vy;
        }
        ofs << "\n";

        s = s1;
        u_prev = u;
    }

    ofs.close();
    std::cout << "Simulation finished. Output: output_dpcbf.csv\n";
    std::cout << "Usage to customize reference speed: ./dpcbf_sim <v_ref>\n";
    return 0;
}