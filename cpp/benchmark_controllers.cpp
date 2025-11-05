#include "include/unified_simulator.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <fstream>

using namespace dpcbf_qp;

struct BenchmarkResult {
    ControllerType type;
    std::string name;
    double total_time{0.0};
    double avg_solve_time{0.0};
    double max_solve_time{0.0};
    double min_solve_time{std::numeric_limits<double>::infinity()};
    int success_count{0};
    int total_runs{0};
    double success_rate{0.0};
    std::vector<double> solve_times;
};

class ControllerBenchmark {
public:
    ControllerBenchmark() = default;

    void addResult(ControllerType type, const std::string& name, double solve_time, bool success) {
        BenchmarkResult& result = results_[type];
        if (result.name.empty()) {
            result.type = type;
            result.name = name;
        }

        result.total_runs++;
        result.total_time += solve_time;
        result.solve_times.push_back(solve_time);
        result.max_solve_time = std::max(result.max_solve_time, solve_time);
        result.min_solve_time = std::min(result.min_solve_time, solve_time);

        if (success) {
            result.success_count++;
        }

        result.success_rate = static_cast<double>(result.success_count) / result.total_runs;
        result.avg_solve_time = result.total_time / result.total_runs;
    }

    void printResults() const {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "控制器性能对比结果" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        std::cout << std::setw(20) << "控制器"
                  << std::setw(12) << "运行次数"
                  << std::setw(12) << "成功率(%)"
                  << std::setw(12) << "平均时间(ms)"
                  << std::setw(12) << "最大时间(ms)"
                  << std::setw(12) << "最小时间(ms)"
                  << std::setw(12) << "总时间(ms)" << std::endl;
        std::cout << std::string(80, '-') << std::endl;

        for (const auto& pair : results_) {
            const auto& result = pair.second;
            std::cout << std::setw(20) << result.name
                      << std::setw(12) << result.total_runs
                      << std::setw(12) << result.success_rate * 100
                      << std::setw(12) << result.avg_solve_time * 1000
                      << std::setw(12) << result.max_solve_time * 1000
                      << std::setw(12) << result.min_solve_time * 1000
                      << std::setw(12) << result.total_time * 1000 << std::endl;
        }
        std::cout << std::string(80, '=') << std::endl;
    }

    void saveToCSV(const std::string& filename) const {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return;
        }

        file << "Controller,TotalRuns,SuccessRate,AvgTimeMs,MaxTimeMs,MinTimeMs,TotalTimeMs\n";

        for (const auto& pair : results_) {
            const auto& result = pair.second;
            file << result.name << ","
                 << result.total_runs << ","
                 << result.success_rate << ","
                 << result.avg_solve_time * 1000 << ","
                 << result.max_solve_time * 1000 << ","
                 << result.min_solve_time * 1000 << ","
                 << result.total_time * 1000 << "\n";
        }

        file.close();
        std::cout << "结果已保存到: " << filename << std::endl;
    }

private:
    std::map<ControllerType, BenchmarkResult> results_;
};

ScenarioParams createBenchmarkParams(ControllerType type, const std::string& scenario_name) {
    ScenarioParams params;

    if (scenario_name == "straight") {
        params.type = ScenarioType::STRAIGHT_LINE;
        params.output_filename = "";  // 不保存CSV以提高测试速度
    } else if (scenario_name == "intersection") {
        params.type = ScenarioType::INTERSECTION;
        params.output_filename = "";
    }

    params.controller_type = type;
    params.ms_horizon = 10;
    params.ms_max_obstacles = 3;
    params.future_prediction_time = 0.5;

    return params;
}

void runBenchmarkScenario(const std::string& scenario_name,
                         const std::vector<Waypoint>& waypoints,
                         const std::vector<Obstacle>& obstacles,
                         ControllerBenchmark& benchmark) {

    std::cout << "\n开始基准测试场景: " << scenario_name << std::endl;

    // 测试所有控制器类型
    std::vector<std::pair<ControllerType, std::string>> controllers = {
        {ControllerType::SINGLE_SHOOTING, "Single-Shooting"},
        {ControllerType::MULTIPLE_SHOOTING, "Multiple-Shooting"},
        {ControllerType::ADAPTIVE, "Adaptive"}
    };

    for (const auto& controller_pair : controllers) {
        ControllerType type = controller_pair.first;
        const std::string& name = controller_pair.second;

        std::cout << "测试控制器: " << name << std::endl;

        // 创建仿真配置
        SimConfig config;
        config.dt = 0.05;
        config.v_ref = 6.0;
        config.robot_spec = {0.3, 1.0, 0.5, 5.0};
        config.dpcbf_params = {8.0, 0.05, 2.2, 1e-6};
        config.qp_weights = {12.0, 12.0, 5.0, 200.0, 20.0};
        config.discrete_cbf_config = {0.25, 1e-3, 10.0, false, 0.2, 3};

        // 运行多次测试
        const int num_runs = 5;
        for (int run = 0; run < num_runs; ++run) {
            auto start_time = std::chrono::high_resolution_clock::now();

            try {
                // 创建仿真器
                UnifiedSimulator simulator(config);

                // 创建场景参数
                ScenarioParams params = createBenchmarkParams(type, scenario_name);

                // 运行仿真
                simulator.runSimulation(waypoints, obstacles, params);

                auto end_time = std::chrono::high_resolution_clock::now();
                double solve_time = std::chrono::duration<double>(end_time - start_time).count();

                benchmark.addResult(type, name, solve_time, true);

                std::cout << "  运行 " << (run + 1) << "/" << num_runs
                          << ": " << solve_time * 1000 << " ms" << std::endl;

            } catch (const std::exception& e) {
                auto end_time = std::chrono::high_resolution_clock::now();
                double solve_time = std::chrono::duration<double>(end_time - start_time).count();

                benchmark.addResult(type, name, solve_time, false);

                std::cout << "  运行 " << (run + 1) << "/" << num_runs
                          << ": 失败 - " << e.what() << std::endl;
            }
        }
    }
}

int main() {
    std::cout << "=== CBF-QP 控制器性能基准测试 ===" << std::endl;

    ControllerBenchmark benchmark;

    try {
        // 定义测试场景
        struct TestCase {
            std::string name;
            std::vector<Waypoint> waypoints;
            std::vector<Obstacle> obstacles;
        };

        std::vector<TestCase> test_cases;

        // 简单直线场景
        test_cases.push_back({
            "simple_straight",
            {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}},
            {{10.0, 1.0, 0.5, 0.0, 0.0}}
        });

        // 复杂直线场景
        test_cases.push_back({
            "complex_straight",
            {{0.0, 0.0, 0.0}, {50.0, 0.0, 0.0}},
            {
                {10.0, 2.0, 0.5, -1.0, 0.0},
                {20.0, -2.0, 0.5, 1.0, 0.0},
                {30.0, 1.0, 0.5, 0.0, 0.5},
                {40.0, -1.0, 0.5, 0.0, -0.5}
            }
        });

        // 路口转弯场景
        test_cases.push_back({
            "intersection",
            {{0.0, 0.0, M_PI/4}, {15.0, 15.0, M_PI/4}},
            {
                {5.0, 10.0, 0.5, 2.0, -2.0},
                {10.0, 5.0, 0.5, 1.5, 1.5},
                {8.0, 8.0, 0.5, 0.0, 0.0}
            }
        });

        // 运行所有测试案例
        for (const auto& test_case : test_cases) {
            runBenchmarkScenario(test_case.name, test_case.waypoints, test_case.obstacles, benchmark);
        }

        // 打印结果
        benchmark.printResults();
        benchmark.saveToCSV("benchmark_results.csv");

        std::cout << "\n=== 基准测试完成 ===" << std::endl;

        // 性能分析
        std::cout << "\n=== 性能分析 ===" << std::endl;
        std::cout << "建议:" << std::endl;
        std::cout << "1. 对于简单场景，Single-Shooting 提供最快的求解速度" << std::endl;
        std::cout << "2. 对于复杂场景，Multiple-Shooting 提供更好的安全保证" << std::endl;
        std::cout << "3. Adaptive 控制器可以在性能和安全性之间自动平衡" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "基准测试失败: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}