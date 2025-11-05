# Multiple-Shooting CBF-QP 控制器

## 概述

本项目实现了基于 Multiple-Shooting 方法的动态抛物线控制屏障函数（DPCBF）控制器，用于解决原始 Single-Shooting 方法在复杂环境下的局限性。

## 主要特性

### 1. Multiple-Shooting 控制器
- **多步预测**：将预测时域分割为多个区间，每步独立优化
- **动力学一致性**：通过等式约束确保节点间的动力学连续性
- **CBF 安全约束**：在每个预测节点强制执行安全约束
- **自适应时域**：根据环境复杂度动态调整预测长度

### 2. 高级 QP 求解器
- **稀疏矩阵支持**：高效处理大规模优化问题
- **热启动**：加速相似问题的求解
- **时间监控**：确保实时性要求
- **解的验证**：自动检查解的可行性

### 3. 自适应控制器选择
- **环境复杂度评估**：基于障碍物密度和相对速度
- **性能监控**：记录各控制器的求解时间和成功率
- **智能切换**：根据场景特征自动选择最适合的控制器

## 文件结构

```
cpp/
├── include/
│   ├── multiple_shooting_controller.h    # Multiple-Shooting 控制器
│   ├── advanced_qp_solver.h              # 高级 QP 求解器
│   └── unified_simulator.h               # 更新的统一仿真器
├── src/
│   ├── multiple_shooting_controller.cpp  # MS 控制器实现
│   ├── advanced_qp_solver.cpp            # QP 求解器实现
│   └── unified_simulator.cpp             # 集成 MS 控制器的仿真器
├── test_multiple_shooting.cpp           # MS 控制器测试程序
├── benchmark_controllers.cpp            # 控制器性能基准测试
└── MULTIPLE_SHOOTING_README.md          # 本文档
```

## 编译和运行

### 1. 编译项目

```bash
mkdir build && cd build
cmake ..
make -j4
```

### 2. 运行 Multiple-Shooting 测试

```bash
# 自适应控制器测试所有场景
./test_multiple_shooting --controller=adaptive --scenario=both

# 测试 Multiple-Shooting 控制器
./test_multiple_shooting --controller=multiple --scenario=straight --ms_horizon=15

# 测试 Single-Shooting 控制器作为对比
./test_multiple_shooting --controller=single --scenario=intersection

# 详细性能统计
./test_multiple_shooting --controller=adaptive --verbose --save_csv=true
```

### 3. 运行性能基准测试

```bash
./benchmark_controllers
```

这将生成 `benchmark_results.csv` 文件，包含详细的性能对比数据。

## 控制器参数配置

### Multiple-Shooting 参数

```cpp
struct MultipleShootingParams {
    int horizon{10};              // 预测时域长度
    double dt{0.05};              // 时间步长
    double weight_control{12.0};   // 控制偏差权重
    double weight_rate{5.0};       // 控制变化率权重
    double weight_terminal{100.0}; // 终端状态权重
    double slack_penalty{20.0};    // 约束违反惩罚
    int max_obstacles{3};          // 最大障碍物数量
    double constraint_tol{1e-4};   // 约束容差
    int max_qp_iterations{4000};   // QP最大迭代次数
};
```

### 场景参数

```cpp
struct ScenarioParams {
    ScenarioType type;                    // 场景类型
    ControllerType controller_type;        // 控制器类型
    int ms_horizon;                       // MS预测时域
    int ms_max_obstacles;                 // MS最大障碍物数
    double ms_weight_control;             // MS控制权重
    double ms_weight_rate;                // MS变化率权重
    // ... 其他参数
};
```

## 使用示例

### 1. 基本使用

```cpp
#include "include/unified_simulator.h"

// 创建配置
SimConfig config = {/* 参数 */};

// 创建仿真器
UnifiedSimulator simulator(config);

// 设置场景参数
ScenarioParams params;
params.controller_type = ControllerType::MULTIPLE_SHOOTING;
params.ms_horizon = 10;
params.ms_max_obstacles = 3;

// 运行仿真
simulator.runSimulation(waypoints, obstacles, params);
```

### 2. 自适应控制器使用

```cpp
ScenarioParams params;
params.controller_type = ControllerType::ADAPTIVE;  // 自动选择控制器

// 仿真器会根据环境复杂度自动选择最合适的控制器
simulator.runSimulation(waypoints, obstacles, params);

// 获取性能统计
auto stats = simulator.getControllerStatistics();
```

### 3. 直接使用 Multiple-Shooting 控制器

```cpp
#include "include/multiple_shooting_controller.h"

// 创建控制器
MultipleShootingParams ms_params;
MultipleShootingController controller(model, dparams, ms_params);

// 求解控制问题
auto result = controller.solve(initial_state, u_ref, obstacles);

if (result.success) {
    Control control = result.control_sequence[0];
    // 使用控制输入
}
```

## 性能优化建议

### 1. 参数调优

- **简单环境**：使用较短的预测时域（5-8步）
- **复杂环境**：使用较长的预测时域（10-15步）
- **实时性要求高**：减少最大障碍物数量

### 2. 控制器选择

| 场景类型 | 推荐控制器 | 原因 |
|---------|-----------|------|
| 简单直线 | Single-Shooting | 计算速度快，满足安全性要求 |
| 复杂避障 | Multiple-Shooting | 更好的安全保证，多步优化 |
| 未知环境 | Adaptive | 自动适应，平衡性能和安全性 |

### 3. 实时性保证

- 设置求解时间限制（建议 < 20ms）
- 使用热启动加速求解
- 在求解失败时回退到 Single-Shooting

## 算法对比

| 特性 | Single-Shooting | Multiple-Shooting |
|------|----------------|------------------|
| 计算复杂度 | 低 | 中等 |
| 数值稳定性 | 中等 | 高 |
| 安全保证 | 局部 | 全局（时域内） |
| 预测能力 | 单步 | 多步 |
| 实时性 | 优秀 | 良好 |
| 适用场景 | 简单环境 | 复杂环境 |

## 故障排除

### 1. 编译错误

- 确保安装了所有依赖：`sudo apt-get install libosqp-dev libeigen3-dev`
- 检查 CMake 版本 >= 3.12
- 确保编译器支持 C++17

### 2. 运行时错误

- **QP 求解失败**：检查约束是否可行，调整参数容差
- **数值不稳定**：增加正则化项，检查矩阵条件数
- **实时性不满足**：减少预测时域或障碍物数量

### 3. 性能问题

- **求解时间过长**：使用自适应控制器或调整参数
- **成功率低**：检查场景设置，增加约束松弛
- **内存使用高**：减少预测时域长度

## 贡献指南

欢迎提交 Issue 和 Pull Request！

### 开发环境设置

1. 安装依赖
2. 编译项目
3. 运行测试确保功能正常
4. 添加新的测试用例

### 代码风格

- 使用中文注释
- 遵循现有的命名约定
- 添加适当的文档说明

## 许可证

本项目遵循与主项目相同的许可证。

---

**版本**: 1.0
**最后更新**: 2024年10月
**作者**: Claude Code Assistant