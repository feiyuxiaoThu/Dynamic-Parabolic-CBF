#include "../include/unified_simulator.h"

namespace dpcbf_qp {

UnifiedSimulator::UnifiedSimulator(const SimConfig& config) 
    : config_(config), model_(config.dt, config.robot_spec),
      controller_(model_, config.dpcbf_params, config.qp_weights, config.discrete_cbf_config) {
}

void UnifiedSimulator::runSimulation(const std::vector<Waypoint>& waypoints, 
                                   const std::vector<Obstacle>& obstacles,
                                   const ScenarioParams& scenario_params) {
    
    // 创建参考轨迹
    auto quinticSpline = createReferenceTrajectory(waypoints, scenario_params);
    
    // 计算原始轨迹的终点时间（不包括延长部分）
    double original_trajectory_time = calculateOriginalTrajectoryTime(waypoints);
    
    // 初始化车辆状态
    State s;
    s.x = waypoints[0].x;
    s.y = waypoints[0].y;
    s.theta = waypoints[0].theta;
    s.v = scenario_params.initial_velocity;
    
    std::vector<Obstacle> sim_obstacles = obstacles;
    
    // 根据是否需要保存CSV来决定是否创建文件
    std::ofstream ofs;
    bool save_to_file = !scenario_params.output_filename.empty();
    last_output_filename_ = scenario_params.output_filename;
    
    if (save_to_file) {
        ofs.open(last_output_filename_);
        writeCSVHeader(ofs, sim_obstacles.size());
    }
    
    // 使用原始轨迹时间作为仿真终止条件，而不是延长后的时间
    double T = original_trajectory_time;
    int steps = static_cast<int>(T / config_.dt);
    
    Control u_prev; 
    u_prev.steer = 0.0; 
    u_prev.a = 0.0;
    
    std::cout << "Starting " << (scenario_params.type == ScenarioType::STRAIGHT_LINE ? "straight line" : "intersection") 
              << " simulation..." << std::endl;
    std::cout << "Total trajectory time: " << T << " seconds" << std::endl;
    
    for (int k = 0; k < steps; ++k) {
        double t = k * config_.dt;
        
        // 更新障碍物位置
        updateObstacles(sim_obstacles, t, config_.dt, scenario_params);
        
        // 获取参考轨迹信息
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
        
        // 计算速度限制
        double kappa_eps = 1e-6;
        double denom = std::pow(std::max(vel_norm, kappa_eps), 3);
        double kappa = (vel_norm > kappa_eps) ? ((vx*ay - vy*ax) / denom) : 0.0;
        double v_cap = (std::abs(kappa) > kappa_eps) ? std::sqrt(scenario_params.a_lat_max / std::abs(kappa)) : config_.v_ref;
        double desired_v = std::min(config_.v_ref, v_cap);
        
        // 计算回到参考轨迹的目标轨迹
        auto return_spline = computeReturnTrajectory(s, quinticSpline, t);
        
        // 计算未来时刻的目标控制量
        Control u_future = computeFutureControl(return_spline, scenario_params.future_prediction_time, config_.dt);
        
        // 使用未来控制量作为参考
        Control u_ref;
        u_ref.steer = u_future.steer;
        u_ref.a = u_future.a;
        
        // 更新QP控制器的上一时刻控制输入（用于jerk惩罚）
        controller_.updatePreviousControl(u_prev);
        
        Control u_raw = controller_.solve(s, u_ref, sim_obstacles);
        
        // 应用控制限制
        double steer_cmd = BicycleModel::clamp(
            u_prev.steer + BicycleModel::clamp(u_raw.steer - u_prev.steer, -scenario_params.dsteer_max, scenario_params.dsteer_max),
            -config_.robot_spec.steer_max, config_.robot_spec.steer_max
        );
        double a_cmd = BicycleModel::clamp(
            u_prev.a + BicycleModel::clamp(u_raw.a - u_prev.a, -scenario_params.da_max_step, scenario_params.da_max_step),
            -config_.robot_spec.a_max, config_.robot_spec.a_max
        );
        Control u; 
        u.steer = steer_cmd; 
        u.a = a_cmd;
        
        // 更新车辆状态
        State s1 = model_.step(s, u);
        
        // 计算安全距离
        double hmin = 1e9;
        for (const auto& ob : sim_obstacles) {
            auto hres = dpcbf_continuous(s.x, s.y, s.theta, s.v, ob, config_.robot_spec.radius, config_.dpcbf_params);
            hmin = std::min(hmin, hres.h);
        }
        
        // 写入CSV（仅在需要保存文件时）
        if (save_to_file) {
            writeCSVRow(ofs, t, s, u, hmin, x_ref, y_ref, sim_obstacles);
        }
        
        s = s1;
        u_prev = u;
        
        // 输出关键时刻的信息
        if (k % 100 == 0) {
            std::cout << "t=" << t << "s, pos=(" << s.x << "," << s.y << "), v=" << s.v 
                      << ", h_min=" << hmin << std::endl;
        }
    }
    
    if (save_to_file) {
        ofs.close();
        std::cout << "Simulation finished. Output: " << last_output_filename_ << std::endl;
    } else {
        std::cout << "Simulation finished. (No CSV output)" << std::endl;
    }
}

double UnifiedSimulator::calculateOriginalTrajectoryTime(const std::vector<Waypoint>& waypoints) {
    // 计算原始轨迹（不包括延长部分）的总时间
    double total_time = 0.0;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        double dx = waypoints[i+1].x - waypoints[i].x;
        double dy = waypoints[i+1].y - waypoints[i].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        // 根据场景类型调整速度
        double speed_factor = 1.0;
        if (i >= 1 && i <= waypoints.size()-3) { // 中间段
            speed_factor = 0.6; // 可能的转弯段降速
        }
        
        double segment_time = dist / std::max(1e-6, config_.v_ref * speed_factor);
        total_time += segment_time;
    }
    
    return total_time;
}

SplineTrajectory::QuinticSpline3D UnifiedSimulator::createReferenceTrajectory(
    const std::vector<Waypoint>& waypoints, 
    const ScenarioParams& scenario_params) {
    
    // 延长参考轨迹：在终点后添加额外的路径点
    std::vector<Waypoint> extended_waypoints = waypoints;
    
    if (waypoints.size() >= 2) {
        Waypoint last_point = waypoints.back();
        Waypoint second_last = waypoints[waypoints.size() - 2];
        
        // 计算最后一段的方向
        double dx = last_point.x - second_last.x;
        double dy = last_point.y - second_last.y;
        double segment_length = std::sqrt(dx*dx + dy*dy);
        
        if (segment_length > 1e-6) {
            // 标准化方向向量
            dx /= segment_length;
            dy /= segment_length;
            
            // 延长距离：确保有足够的时间进行轨迹回归
            double extension_distance = config_.v_ref * 3.0; // 延长3秒的距离
            
            // 添加延长的路径点
            for (int i = 1; i <= 3; ++i) {
                Waypoint extended_point;
                extended_point.x = last_point.x + dx * (extension_distance / 3.0) * i;
                extended_point.y = last_point.y + dy * (extension_distance / 3.0) * i;
                extended_point.theta = last_point.theta; // 保持相同的朝向
                extended_waypoints.push_back(extended_point);
            }
        }
    }
    
    // 构建路径点（使用延长后的轨迹）
    Eigen::MatrixXd route(3, static_cast<int>(extended_waypoints.size()));
    for (int i = 0; i < (int)extended_waypoints.size(); ++i) {
        route(0, i) = extended_waypoints[i].x;
        route(1, i) = extended_waypoints[i].y;
        route(2, i) = 0.0;
    }
    
    // 计算时间分段（使用延长后的轨迹）
    Eigen::VectorXd ts(static_cast<int>(extended_waypoints.size()) - 1);
    int original_segments = waypoints.size() - 1;
    
    for (int i = 0; i < ts.size(); ++i) {
        double dx = route(0, i+1) - route(0, i);
        double dy = route(1, i+1) - route(1, i);
        double dist = std::sqrt(dx*dx + dy*dy);
        
        // 根据场景类型和段位置调整速度
        double speed_factor = 1.0;
        
        if (i < original_segments) {
            // 原始轨迹段
            if (scenario_params.type == ScenarioType::INTERSECTION) {
                // 在转弯段降低速度
                if (i >= 1 && i <= original_segments-2) {
                    speed_factor = 0.6;
                }
            }
        } else {
            // 延长轨迹段：保持匀速
            speed_factor = 1.0;
        }
        
        ts(i) = dist / std::max(1e-6, config_.v_ref * speed_factor);
    }
    
    // 创建样条点
    SplineTrajectory::SplineVector3D spline_points;
    spline_points.reserve(route.cols());
    for (int i = 0; i < route.cols(); ++i) {
        SplineTrajectory::SplinePoint3d p; 
        p << route(0,i), route(1,i), route(2,i);
        spline_points.push_back(p);
    }
    
    std::vector<double> time_segments(ts.size());
    for (int i = 0; i < ts.size(); ++i) time_segments[i] = ts(i);
    
    // 边界条件
    SplineTrajectory::BoundaryConditions<3> boundary;
    boundary.start_velocity.setZero();
    boundary.start_acceleration.setZero();
    boundary.end_velocity.setZero();
    boundary.end_acceleration.setZero();
    
    SplineTrajectory::QuinticSpline3D quinticSpline;
    quinticSpline.update(time_segments, spline_points, 0.0, boundary);
    
    return quinticSpline;
}

SplineTrajectory::QuinticSpline3D UnifiedSimulator::computeReturnTrajectory(
    const State& state, 
    const SplineTrajectory::QuinticSpline3D& ref_spline, 
    double t, 
    double duration) {
    
    // 由于参考轨迹已经延长，不需要复杂的终点策略
    double ref_duration = ref_spline.getTrajectory().getDuration();
    
    // 找到参考轨迹上最近的点
    double closest_t = std::max(0.0, std::min(t, ref_duration));
    double min_dist = 1e9;
    
    // 搜索最近点（扩大搜索范围，因为轨迹已延长）
    double search_start = std::max(0.0, t - 1.0);
    double search_end = std::min(ref_duration, t + 3.0); // 可以搜索更远的范围
    
    for (double search_t = search_start; search_t <= search_end; search_t += 0.1) {
        auto ref_pos = ref_spline.getTrajectory().getPos(search_t);
        double dist = std::sqrt(std::pow(ref_pos(0) - state.x, 2) + 
                               std::pow(ref_pos(1) - state.y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest_t = search_t;
        }
    }
    
    // 创建回到参考轨迹的路径点
    SplineTrajectory::SplineVector3D return_points;
    std::vector<double> time_segments;
    
    // 起点：当前位置
    SplineTrajectory::SplinePoint3d start_point;
    start_point << state.x, state.y, 0.0;
    return_points.push_back(start_point);
    
    // 中间点：参考轨迹上的目标点
    double mid_t = std::min(ref_duration, closest_t + duration/2);
    auto target_pos = ref_spline.getTrajectory().getPos(mid_t);
    SplineTrajectory::SplinePoint3d mid_point;
    mid_point << target_pos(0), target_pos(1), 0.0;
    return_points.push_back(mid_point);
    
    // 终点：参考轨迹上更远的点
    double end_t = std::min(ref_duration, closest_t + duration);
    auto end_pos = ref_spline.getTrajectory().getPos(end_t);
    SplineTrajectory::SplinePoint3d end_point;
    end_point << end_pos(0), end_pos(1), 0.0;
    return_points.push_back(end_point);
    
    // 时间分段
    time_segments.push_back(duration / 2.0);
    time_segments.push_back(duration / 2.0);
    
    // 边界条件
    SplineTrajectory::BoundaryConditions<3> boundary;
    boundary.start_velocity << state.v * std::cos(state.theta), 
                              state.v * std::sin(state.theta), 0.0;
    
    // 终点速度
    auto ref_vel = ref_spline.getTrajectory().getVel(end_t);
    boundary.end_velocity << ref_vel(0), ref_vel(1), 0.0;
    boundary.start_acceleration.setZero();
    boundary.end_acceleration.setZero();
    
    // 创建样条轨迹
    SplineTrajectory::QuinticSpline3D return_spline;
    return_spline.update(time_segments, return_points, 0.0, boundary);
    
    return return_spline;
}

Control UnifiedSimulator::computeFutureControl(
    const SplineTrajectory::QuinticSpline3D& spline, 
    double future_time, 
    double dt) {
    
    Control u;
    u.steer = 0.0;
    u.a = 0.0;
    
    double spline_duration = spline.getTrajectory().getDuration();
    if (future_time >= spline_duration) {
        return u;
    }
    
    // 获取未来时刻的位置、速度、加速度
    auto pos = spline.getTrajectory().getPos(future_time);
    auto vel = spline.getTrajectory().getVel(future_time);
    auto acc = spline.getTrajectory().getAcc(future_time);
    
    double vx = vel(0), vy = vel(1);
    double ax = acc(0), ay = acc(1);
    double vel_norm = std::sqrt(vx*vx + vy*vy);
    
    if (vel_norm > 1e-6) {
        // 计算曲率
        double kappa = (vx*ay - vy*ax) / std::pow(vel_norm, 3);
        
        // 计算转向角
        double wheelbase = model_.spec().L;
        u.steer = std::atan(wheelbase * kappa);
        
        // 计算加速度
        u.a = std::sqrt(ax*ax + ay*ay);
        if (vx*ax + vy*ay < 0) u.a = -u.a; // 如果是减速
    }
    
    return u;
}

void UnifiedSimulator::updateObstacles(std::vector<Obstacle>& obstacles, 
                                     double t, 
                                     double dt, 
                                     const ScenarioParams& scenario_params) {
    
    if (scenario_params.type == ScenarioType::STRAIGHT_LINE) {
        // 直线场景：简单的线性运动
        for (auto& ob : obstacles) {
            ob.ox += ob.vx * dt;
            ob.oy += ob.vy * dt;
        }
    } else if (scenario_params.type == ScenarioType::INTERSECTION) {
        // 路口场景：复杂的运动模式
        for (size_t i = 0; i < obstacles.size(); ++i) {
            auto& ob = obstacles[i];
            
            if (i == 0) {
                // 车辆1：直行通过路口
                ob.ox += ob.vx * dt;
                ob.oy += ob.vy * dt;
            } else if (i == 1) {
                // 车辆2：从侧面接近路口
                ob.ox += ob.vx * dt;
                ob.oy += ob.vy * dt;
            } else if (i == 2) {
                // 车辆3：对向左转车辆
                ob.ox += ob.vx * dt;
                ob.oy += ob.vy * dt;
                
                // 在特定时间后开始左转
                if (t > 3.0) {
                    double turn_rate = 0.1;
                    double new_vx = ob.vx * cos(turn_rate * dt) - ob.vy * sin(turn_rate * dt);
                    double new_vy = ob.vx * sin(turn_rate * dt) + ob.vy * cos(turn_rate * dt);
                    ob.vx = new_vx;
                    ob.vy = new_vy;
                }
            }
        }
    }
}

void UnifiedSimulator::writeCSVHeader(std::ofstream& ofs, size_t num_obstacles) {
    ofs << "t,x,y,theta,v,steer,a,h_min,ref_x,ref_y";
    for (size_t i = 0; i < num_obstacles; ++i) {
        ofs << ",obs" << i << "_ox,obs" << i << "_oy,obs" << i << "_r,obs" << i << "_vx,obs" << i << "_vy";
    }
    ofs << "\n";
}

void UnifiedSimulator::writeCSVRow(std::ofstream& ofs, double t, const State& state, 
                                 const Control& control, double h_min, 
                                 double ref_x, double ref_y, 
                                 const std::vector<Obstacle>& obstacles) {
    ofs << t << "," << state.x << "," << state.y << "," << state.theta << "," << state.v
        << "," << control.steer << "," << control.a << "," << h_min << "," << ref_x << "," << ref_y;
    for (const auto& ob : obstacles) {
        ofs << "," << ob.ox << "," << ob.oy << "," << ob.r << "," << ob.vx << "," << ob.vy;
    }
    ofs << "\n";
}
} // namespace dpcbf_qp
