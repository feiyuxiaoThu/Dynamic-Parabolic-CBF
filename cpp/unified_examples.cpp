#include <iostream>
#include <vector>
#include <cmath>
#include "include/unified_simulator.h"
#include "include/gflags_config.h"
#include <gflags/gflags.h>

using namespace dpcbf_qp;

// 定义场景选择标志
DEFINE_string(scenario, "straight", "Scenario type: 'straight' or 'intersection'");
DEFINE_bool(save_csv, true, "Whether to save simulation results to CSV file");

int main(int argc, char** argv) {
    // Initialize gflags
    google::SetUsageMessage("Unified Dynamic Parabolic CBF Simulation");
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    // 加载配置参数
    SimConfig config = GFlagsConfig::loadConfig();
    
    // 创建统一仿真器
    UnifiedSimulator simulator(config);
    
    // 输出系统参数
    std::cout << "=== System Configuration ===" << std::endl;
    std::cout << "Robot parameters:" << std::endl;
    std::cout << "  Radius: " << config.robot_spec.radius << std::endl;
    std::cout << "  Max acceleration: " << config.robot_spec.a_max << std::endl;
    std::cout << "  Max steering: " << config.robot_spec.steer_max << std::endl;
    std::cout << "  Wheelbase: " << config.robot_spec.L << std::endl;
    std::cout << "DPCBF parameters:" << std::endl;
    std::cout << "  Margin: " << config.dpcbf_params.margin << std::endl;
    std::cout << "  k_lambda: " << config.dpcbf_params.k_lambda << std::endl;
    std::cout << "  k_mu: " << config.dpcbf_params.k_mu << std::endl;
    std::cout << std::endl;
    
    // 统一调用方式 - 根据场景类型创建不同的配置
    std::string scenario_type = FLAGS_scenario;
    
    if (scenario_type == "straight") {
        // 直线避障场景配置
        ScenarioParams params;
        params.type = ScenarioType::STRAIGHT_LINE;
        params.output_filename = FLAGS_save_csv ? "output_dpcbf.csv" : "";
        params.future_prediction_time = 0.5;
        params.dsteer_max = 0.05;
        params.da_max_step = 0.5;
        params.a_lat_max = 2.0;
        params.initial_velocity = 5.0;
        
        std::vector<Waypoint> waypoints = {
            {1.0, 7.5, 0.0},
            {50.0, 7.5, 0.0}
        };
        
        std::vector<Obstacle> obstacles = {
            {8.0, 9.0, 1.5, 1.5, -2.5},
            {15.0, 4.0, 1.5, -2.5, 1.5},
            {24.0, 5.0, 1.5, 1.5, -2.5},
            {34.0, 9.0, 1.5, 2.5, -1.5}
        };
        
        std::cout << "=== Straight Line Avoidance Scenario ===" << std::endl;
        simulator.runSimulation(waypoints, obstacles, params);
        
    } else if (scenario_type == "intersection") {
        // 路口右转场景配置
        ScenarioParams params;
        params.type = ScenarioType::INTERSECTION;
        params.output_filename = FLAGS_save_csv ? "intersection_output.csv" : "";
        params.future_prediction_time = 0.4;
        params.dsteer_max = 0.08;
        params.da_max_step = 0.6;
        params.a_lat_max = 3.0;
        params.initial_velocity = 5.0;
        
        std::vector<Waypoint> waypoints;
        double radius = 15.0;
        int num_points = 16;
        
        for (int i = 0; i <= num_points; i++) {
            double t = (M_PI / 2.0) * i / num_points;
            double x = radius * sin(t);
            double y = radius * (1.0 - cos(t));
            double theta = t;
            waypoints.push_back({x, y, theta});
        }
        
        std::vector<Obstacle> obstacles = {
            {8.0, 5.0, 1.2, 1.0, 1.0},
            {5.0, 12.0, 1.2, 2.0, -1.0},
            {12.0, 18.0, 1.2, 0.5, -2.0}
        };
        
        std::cout << "=== Right Turn Intersection Scenario ===" << std::endl;
        simulator.runSimulation(waypoints, obstacles, params);
        
    } else if (scenario_type == "both") {
        // 运行两个场景
        std::cout << "=== Running Both Scenarios ===" << std::endl;
        
        // 直线场景
        {
            ScenarioParams params;
            params.type = ScenarioType::STRAIGHT_LINE;
            params.output_filename = FLAGS_save_csv ? "output_dpcbf.csv" : "";
            params.future_prediction_time = 0.5;
            params.dsteer_max = 0.05;
            params.da_max_step = 0.5;
            params.a_lat_max = 2.0;
            params.initial_velocity = 1.0;
            
            std::vector<Waypoint> waypoints = {{1.0, 7.5, 0.0}, {50.0, 7.5, 0.0}};
            std::vector<Obstacle> obstacles = {
                {8.0, 9.0, 1.5, 1.5, -2.5}, {15.0, 4.0, 1.5, -2.5, 1.5},
                {24.0, 5.0, 1.5, 1.5, -2.5}, {34.0, 9.0, 1.5, 2.5, -1.5}
            };
            
            std::cout << "\n=== Straight Line Scenario ===" << std::endl;
            simulator.runSimulation(waypoints, obstacles, params);
        }
        
        std::cout << std::endl;
        
        // 路口场景
        {
            ScenarioParams params;
            params.type = ScenarioType::INTERSECTION;
            params.output_filename = FLAGS_save_csv ? "intersection_output.csv" : "";
            params.future_prediction_time = 0.4;
            params.dsteer_max = 0.08;
            params.da_max_step = 0.6;
            params.a_lat_max = 3.0;
            params.initial_velocity = 2.0;
            
            std::vector<Waypoint> waypoints;
            double radius = 15.0;
            int num_points = 16;
            
            for (int i = 0; i <= num_points; i++) {
                double t = (M_PI / 2.0) * i / num_points;
                double x = radius * sin(t);
                double y = radius * (1.0 - cos(t));
                double theta = t;
                waypoints.push_back({x, y, theta});
            }
            
            std::vector<Obstacle> obstacles = {
                {8.0, 5.0, 1.2, 1.0, 1.0},
                {5.0, 12.0, 1.2, 2.0, -1.0},
                {12.0, 18.0, 1.2, 0.5, -2.0}
            };
            
            std::cout << "\n=== Intersection Scenario ===" << std::endl;
            simulator.runSimulation(waypoints, obstacles, params);
        }
        
    } else {
        std::cerr << "Unknown scenario type: " << scenario_type << std::endl;
        std::cerr << "Available options: 'straight', 'intersection', 'both'" << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Simulation Complete ===" << std::endl;
    if (FLAGS_save_csv) {
        std::cout << "CSV files saved. Use visualization scripts to analyze results:" << std::endl;
        std::cout << "  python viz/unified_visualizer.py --scenario " << scenario_type << std::endl;
    } else {
        std::cout << "Simulation completed without CSV output (--save_csv=false)" << std::endl;
    }
    
    return 0;
}