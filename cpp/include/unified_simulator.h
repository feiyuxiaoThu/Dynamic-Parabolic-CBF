#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include "bicycle_model.h"
#include "dpcbf.h"
#include "qp_controller.h"
#include "SplineTrajectory.hpp"
#include "gflags_config.h"

namespace dpcbf_qp {

// 场景类型枚举
enum class ScenarioType {
    STRAIGHT_LINE,
    INTERSECTION
};

// 场景参数结构体
struct ScenarioParams {
    ScenarioType type;
    std::string output_filename;
    double future_prediction_time;
    double dsteer_max;
    double da_max_step;
    double a_lat_max;
    double initial_velocity;
    
    // 默认构造函数
    ScenarioParams() : 
        type(ScenarioType::STRAIGHT_LINE),
        output_filename("simulation_output.csv"),
        future_prediction_time(0.5),
        dsteer_max(0.05),
        da_max_step(0.5),
        a_lat_max(2.0),
        initial_velocity(1.0) {}
};

// 统一仿真器类
class UnifiedSimulator {
public:
    UnifiedSimulator(const SimConfig& config);
    
    // 主要仿真接口
    void runSimulation(const std::vector<Waypoint>& waypoints, 
                      const std::vector<Obstacle>& obstacles,
                      const ScenarioParams& scenario_params);
    
    // 获取仿真结果
    const std::string& getOutputFilename() const { return last_output_filename_; }
    
private:
    // 核心功能函数
    double calculateOriginalTrajectoryTime(const std::vector<Waypoint>& waypoints);
    
    SplineTrajectory::QuinticSpline3D createReferenceTrajectory(
        const std::vector<Waypoint>& waypoints, 
        const ScenarioParams& scenario_params);
    
    SplineTrajectory::QuinticSpline3D computeReturnTrajectory(
        const State& state, 
        const SplineTrajectory::QuinticSpline3D& ref_spline, 
        double t, 
        double duration = 2.0);
    
    Control computeFutureControl(
        const SplineTrajectory::QuinticSpline3D& spline, 
        double future_time, 
        double dt);
    
    void updateObstacles(std::vector<Obstacle>& obstacles, 
                        double t, 
                        double dt, 
                        const ScenarioParams& scenario_params);
    
    void writeCSVHeader(std::ofstream& ofs, size_t num_obstacles);
    void writeCSVRow(std::ofstream& ofs, double t, const State& state, 
                    const Control& control, double h_min, 
                    double ref_x, double ref_y, 
                    const std::vector<Obstacle>& obstacles);
    
    // 成员变量
    SimConfig config_;
    BicycleModel model_;
    QPController controller_;
    std::string last_output_filename_;
};

} // namespace dpcbf_qp