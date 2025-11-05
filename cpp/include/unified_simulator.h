#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include <limits>
#include <algorithm>
#include "bicycle_model.h"
#include "dpcbf.h"
#include "qp_controller.h"
#include "multiple_shooting_controller.h"
#include "SplineTrajectory.hpp"
#include "gflags_config.h"

namespace dpcbf_qp {

// 场景类型枚举
enum class ScenarioType {
    STRAIGHT_LINE,
    INTERSECTION
};

// 控制器类型枚举
enum class ControllerType {
    SINGLE_SHOOTING,
    MULTIPLE_SHOOTING,
    ADAPTIVE        // 自适应选择控制器
};

// 场景参数结构体
struct ScenarioParams {
    ScenarioType type;
    ControllerType controller_type;
    std::string output_filename;
    double future_prediction_time;
    double dsteer_max;
    double da_max_step;
    double a_lat_max;
    double initial_velocity;

    // Multiple-Shooting 特定参数
    int ms_horizon;
    double ms_dt;
    double ms_weight_control;
    double ms_weight_rate;
    int ms_max_obstacles;

    // 默认构造函数
    ScenarioParams() :
        type(ScenarioType::STRAIGHT_LINE),
        controller_type(ControllerType::ADAPTIVE),
        output_filename("simulation_output.csv"),
        future_prediction_time(0.5),
        dsteer_max(0.05),
        da_max_step(0.5),
        a_lat_max(2.0),
        initial_velocity(1.0),
        ms_horizon(10),
        ms_dt(0.05),
        ms_weight_control(12.0),
        ms_weight_rate(5.0),
        ms_max_obstacles(3) {}
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

    // 获取控制器性能统计
    std::vector<double> getControllerStatistics() const;

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

    // 控制器选择和管理
    ControllerType selectControllerType(const State& state,
                                      const std::vector<Obstacle>& obstacles,
                                      double current_time);

    Control solveWithSingleShooting(const State& state,
                                   const Control& u_ref,
                                   const std::vector<Obstacle>& obstacles);

    Control solveWithMultipleShooting(const State& state,
                                     const Control& u_ref,
                                     const std::vector<Obstacle>& obstacles,
                                     const ScenarioParams& scenario_params);

    // 性能监控
    void updatePerformanceStatistics(ControllerType type, double solve_time, bool success);

    // 成员变量
    SimConfig config_;
    BicycleModel model_;
    QPController controller_;
    std::unique_ptr<MultipleShootingController> ms_controller_;
    std::string last_output_filename_;

    // 性能统计
    struct ControllerStats {
        int usage_count{0};
        double total_solve_time{0.0};
        int success_count{0};
        double max_solve_time{0.0};
        double min_solve_time{std::numeric_limits<double>::infinity()};
    };

    std::map<ControllerType, ControllerStats> controller_stats_;

    // 自适应选择参数
    double complexity_threshold_{5.0};      // 环境复杂度阈值
    double time_budget_{0.02};              // 时间预算（秒）
    int switch_counter_{0};                 // 控制器切换计数器
};

} // namespace dpcbf_qp