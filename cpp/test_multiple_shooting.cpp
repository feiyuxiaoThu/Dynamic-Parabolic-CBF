#include "include/unified_simulator.h"
#include "include/gflags_config.h"
#include <iostream>
#include <chrono>

using namespace dpcbf_qp;

void printUsage() {
    std::cout << "Multiple-Shooting CBF-QP 测试程序" << std::endl;
    std::cout << "用法：" << std::endl;
    std::cout << "  ./test_multiple_shooting [选项]" << std::endl;
    std::cout << "选项：" << std::endl;
    std::cout << "  --controller=TYPE     控制器类型: single, multiple, adaptive (默认: adaptive)" << std::endl;
    std::cout << "  --scenario=TYPE       场景类型: straight, intersection, both (默认: both)" << std::endl;
    std::cout << "  --ms_horizon=N        Multiple-Shooting 预测时域 (默认: 10)" << std::endl;
    std::cout << "  --ms_obstacles=N      最大障碍物数量 (默认: 3)" << std::endl;
    std::cout << "  --save_csv=BOOL       是否保存CSV文件 (默认: true)" << std::endl;
    std::cout << "  --verbose             显示详细信息" << std::endl;
    std::cout << "  --help                显示此帮助信息" << std::endl;
}

void printStatistics(const UnifiedSimulator& simulator, ControllerType type) {
    auto stats = simulator.getControllerStatistics();

    std::string type_name;
    switch (type) {
        case ControllerType::SINGLE_SHOOTING: type_name = "Single-Shooting"; break;
        case ControllerType::MULTIPLE_SHOOTING: type_name = "Multiple-Shooting"; break;
        case ControllerType::ADAPTIVE: type_name = "Adaptive"; break;
    }

    std::cout << "\n=== " << type_name << " 控制器性能统计 ===" << std::endl;

    if (stats.empty()) {
        std::cout << "暂无统计数据" << std::endl;
        return;
    }

    // 每8个值对应一个控制器的统计信息
    for (size_t i = 0; i < stats.size(); i += 8) {
        if (i + 7 < stats.size()) {
            int controller_type = static_cast<int>(stats[i]);
            int usage_count = static_cast<int>(stats[i + 1]);
            double total_time = stats[i + 2];
            double avg_time = stats[i + 3];
            int success_count = static_cast<int>(stats[i + 4]);
            double success_rate = stats[i + 5];
            double max_time = stats[i + 6];
            double min_time = stats[i + 7];

            std::string controller_name;
            switch (controller_type) {
                case 0: controller_name = "Single-Shooting"; break;
                case 1: controller_name = "Multiple-Shooting"; break;
                case 2: controller_name = "Adaptive"; break;
                default: controller_name = "Unknown"; break;
            }

            std::cout << "控制器: " << controller_name << std::endl;
            std::cout << "  使用次数: " << usage_count << std::endl;
            std::cout << "  成功次数: " << success_count << std::endl;
            std::cout << "  成功率: " << success_rate * 100 << "%" << std::endl;
            std::cout << "  平均求解时间: " << avg_time * 1000 << " ms" << std::endl;
            std::cout << "  最大求解时间: " << max_time * 1000 << " ms" << std::endl;
            std::cout << "  最小求解时间: " << min_time * 1000 << " ms" << std::endl;
            std::cout << "  总求解时间: " << total_time * 1000 << " ms" << std::endl;
            std::cout << std::endl;
        }
    }
}

ScenarioParams createScenarioParams(const std::string& scenario_type,
                                   ControllerType controller_type,
                                   const bool& save_csv,
                                   int ms_horizon, int ms_obstacles) {
    ScenarioParams params;

    if (scenario_type == "straight") {
        params.type = ScenarioType::STRAIGHT_LINE;
        params.output_filename = save_csv ? "test_ms_straight_output.csv" : "";
    } else if (scenario_type == "intersection") {
        params.type = ScenarioType::INTERSECTION;
        params.output_filename = save_csv ? "test_ms_intersection_output.csv" : "";
    }

    params.controller_type = controller_type;
    params.ms_horizon = ms_horizon;
    params.ms_max_obstacles = ms_obstacles;

    return params;
}

void runSingleScenario(const std::string& scenario_name,
                      const std::vector<Waypoint>& waypoints,
                      const std::vector<Obstacle>& obstacles,
                      ControllerType controller_type,
                      int ms_horizon, int ms_obstacles,
                      bool save_csv, bool verbose) {

    std::cout << "\n--- 运行场景: " << scenario_name << " ---" << std::endl;
    std::cout << "控制器类型: ";
    switch (controller_type) {
        case ControllerType::SINGLE_SHOOTING: std::cout << "Single-Shooting"; break;
        case ControllerType::MULTIPLE_SHOOTING: std::cout << "Multiple-Shooting"; break;
        case ControllerType::ADAPTIVE: std::cout << "Adaptive"; break;
    }
    std::cout << std::endl;

    if (controller_type == ControllerType::MULTIPLE_SHOOTING || controller_type == ControllerType::ADAPTIVE) {
        std::cout << "MS参数: 时域=" << ms_horizon << ", 最大障碍物=" << ms_obstacles << std::endl;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    // 创建仿真配置
    SimConfig config;
    config.dt = 0.05;
    config.v_ref = 6.0;
    config.robot_spec = {0.3, 1.0, 0.5, 5.0};
    config.dpcbf_params = {8.0, 0.05, 2.2, 1e-6};
    config.qp_weights = {12.0, 12.0, 5.0, 200.0, 20.0};
    config.discrete_cbf_config = {0.25, 1e-3, 10.0, false, 0.2, 3};

    // 创建仿真器
    UnifiedSimulator simulator(config);

    // 创建场景参数
    ScenarioParams scenario_params = createScenarioParams(
        scenario_name, controller_type, save_csv, ms_horizon, ms_obstacles);

    // 运行仿真
    simulator.runSimulation(waypoints, obstacles, scenario_params);

    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "仿真完成，总时间: " << total_time << " 秒" << std::endl;
    std::cout << "输出文件: " << simulator.getOutputFilename() << std::endl;

    if (verbose) {
        printStatistics(simulator, controller_type);
    }
}

int main(int argc, char* argv[]) {
    // 默认参数
    std::string controller_str = "adaptive";
    std::string scenario_str = "both";
    int ms_horizon = 10;
    int ms_obstacles = 3;
    bool save_csv = true;
    bool verbose = false;

    // 简单的参数解析
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help") {
            printUsage();
            return 0;
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg.find("--controller=") == 0) {
            controller_str = arg.substr(13);
        } else if (arg.find("--scenario=") == 0) {
            scenario_str = arg.substr(11);
        } else if (arg.find("--ms_horizon=") == 0) {
            ms_horizon = std::stoi(arg.substr(14));
        } else if (arg.find("--ms_obstacles=") == 0) {
            ms_obstacles = std::stoi(arg.substr(16));
        } else if (arg.find("--save_csv=") == 0) {
            std::string bool_str = arg.substr(11);
            save_csv = (bool_str == "true" || bool_str == "1");
        }
    }

    // 解析控制器类型
    ControllerType controller_type;
    if (controller_str == "single") {
        controller_type = ControllerType::SINGLE_SHOOTING;
    } else if (controller_str == "multiple") {
        controller_type = ControllerType::MULTIPLE_SHOOTING;
    } else if (controller_str == "adaptive") {
        controller_type = ControllerType::ADAPTIVE;
    } else {
        std::cerr << "错误: 未知的控制器类型 '" << controller_str << "'" << std::endl;
        printUsage();
        return 1;
    }

    std::cout << "=== Multiple-Shooting CBF-QP 测试程序 ===" << std::endl;
    std::cout << "控制器类型: " << controller_str << std::endl;
    std::cout << "测试场景: " << scenario_str << std::endl;
    std::cout << "MS时域: " << ms_horizon << std::endl;
    std::cout << "最大障碍物数: " << ms_obstacles << std::endl;
    std::cout << "保存CSV: " << (save_csv ? "是" : "否") << std::endl;

    try {
        // 定义直线场景
        std::vector<Waypoint> straight_waypoints = {
            {1.0, 7.5, 0.0},
            {50.0, 7.5, 0.0}
        };

        std::vector<Obstacle> straight_obstacles = {
            {10.0, 5.0, 0.5, 0.0, 0.0},     // 静止障碍物
            {20.0, 10.0, 0.5, -1.0, 0.0},   // 左移障碍物
            {30.0, 7.5, 0.5, 0.0, 0.5},     // 上移障碍物
            {40.0, 6.0, 0.5, 0.0, -0.5}     // 下移障碍物
        };

        // 定义路口场景
        std::vector<Waypoint> intersection_waypoints = {
            {0.0, 0.0, M_PI/4},
            {15.0, 15.0, M_PI/4}
        };

        std::vector<Obstacle> intersection_obstacles = {
            {5.0, 10.0, 0.5, 2.0, -2.0},    // 对向车辆
            {10.0, 5.0, 0.5, 1.5, 1.5},     // 侧向车辆
            {8.0, 8.0, 0.5, 0.0, 0.0}       // 静止车辆
        };

        // 运行测试场景
        if (scenario_str == "straight" || scenario_str == "both") {
            runSingleScenario("straight", straight_waypoints, straight_obstacles,
                            controller_type, ms_horizon, ms_obstacles, save_csv, verbose);
        }

        if (scenario_str == "intersection" || scenario_str == "both") {
            runSingleScenario("intersection", intersection_waypoints, intersection_obstacles,
                            controller_type, ms_horizon, ms_obstacles, save_csv, verbose);
        }

        std::cout << "\n=== 所有测试完成 ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}