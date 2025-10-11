#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "include/bicycle_model.h"
#include "include/dpcbf.h"
#include "include/qp_controller.h"
#include "include/SplineTrajectory.hpp"
#include "include/config_loader.h"

int main(int argc, char** argv) {
    // Path is relative to the build directory
    std::string config_path = "config/sim_config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    SimConfig config = ConfigLoader::loadConfig(config_path);

    BicycleModel model(config.dt, config.robot_spec);
    QPController controller(model, config.dpcbf_params, config.qp_weights, config.discrete_cbf_config);

    // 使用 QuinticSpline3D 构造参考轨迹
    Eigen::MatrixXd route(3, static_cast<int>(config.waypoints.size()));
    for (int i = 0; i < (int)config.waypoints.size(); ++i) {
        route(0, i) = config.waypoints[i].x;
        route(1, i) = config.waypoints[i].y;
        route(2, i) = 0.0; // Z-axis is 0 for 2D simulation
    }
    Eigen::VectorXd ts(static_cast<int>(config.waypoints.size()) - 1);
    for (int i = 0; i < ts.size(); ++i) {
        double dx = route(0, i+1) - route(0, i);
        double dy = route(1, i+1) - route(1, i);
        double dist = std::sqrt(dx*dx + dy*dy);
        ts(i) = dist / std::max(1e-6, config.v_ref);
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
    s.x = config.waypoints[0].x;
    s.y = config.waypoints[0].y;
    s.theta = config.waypoints[0].theta;
    s.v = 1.0;

    std::vector<Obstacle> obstacles = config.obstacles;

    std::ofstream ofs("output_dpcbf.csv");
    ofs << "t,x,y,theta,v,steer,a,h_min,ref_x,ref_y";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        ofs << ",obs" << i << "_ox,obs" << i << "_oy,obs" << i << "_r,obs" << i << "_vx,obs" << i << "_vy";
    }
    ofs << "\n";

    double T = quinticSpline.getTrajectory().getDuration();
    int steps = static_cast<int>(T / config.dt);

    Control u_prev; u_prev.steer = 0.0; u_prev.a = 0.0;
    const double dsteer_max = 0.05;
    const double da_max_step = 0.5;

    for (int k=0; k<steps; ++k) {
        double t = k * config.dt;

        for (auto& ob : obstacles) {
            ob.ox += ob.vx * config.dt;
            ob.oy += ob.vy * config.dt;
        }

        double t_eval = std::min(t, quinticSpline.getTrajectory().getDuration());
        auto pos = quinticSpline.getTrajectory().getPos(t_eval);
        auto vel = quinticSpline.getTrajectory().getVel(t_eval);
        auto acc = quinticSpline.getTrajectory().getAcc(t_eval);
        double x_ref = pos(0);
        double y_ref = pos(1);
        double vx = vel(0), vy = vel(1);
        double ax = acc(0), ay = acc(1);
        double vel_norm = std::sqrt(vx*vx + vy*vy);
        double desired_theta = (vel_norm > 1e-6) ? std::atan2(vy, vx) : std::atan2(y_ref - s.y, x_ref - s.x);

        double kappa_eps = 1e-6;
        double denom = std::pow(std::max(vel_norm, kappa_eps), 3);
        double kappa = (vel_norm > kappa_eps) ? ((vx*ay - vy*ax) / denom) : 0.0;
        double a_lat_max = 2.0;
        double v_cap = (std::abs(kappa) > kappa_eps) ? std::sqrt(a_lat_max / std::abs(kappa)) : config.v_ref;
        double desired_v = std::min(config.v_ref, v_cap);

        Control u_ref = model.nominal_track_ref(s, desired_theta, desired_v);
        Control u_raw = controller.solve(s, u_ref, obstacles, x_ref, y_ref, desired_theta);

        double steer_cmd = BicycleModel::clamp(
            u_prev.steer + BicycleModel::clamp(u_raw.steer - u_prev.steer, -dsteer_max, dsteer_max),
            -config.robot_spec.steer_max, config.robot_spec.steer_max
        );
        double a_cmd = BicycleModel::clamp(
            u_prev.a + BicycleModel::clamp(u_raw.a - u_prev.a, -da_max_step, da_max_step),
            -config.robot_spec.a_max, config.robot_spec.a_max
        );
        Control u; u.steer = steer_cmd; u.a = a_cmd;

        State s1 = model.step(s, u);

        double hmin = 1e9;
        for (const auto& ob : obstacles) {
            auto hres = dpcbf_continuous(s.x, s.y, s.theta, s.v, ob, config.robot_spec.radius, config.dpcbf_params);
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
    std::cout << "Usage: ./dpcbf_sim [config_path.yaml]\n";
    return 0;
}