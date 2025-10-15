# Dynamic Parabolic CBF 统一仿真系统

## 概述

本项目实现了基于动态抛物线控制屏障函数(Dynamic Parabolic CBF)的自动驾驶仿真系统，支持多种场景的避障和轨迹跟踪，具备避障后自动回到参考轨迹的能力。

## 主要特性

### 🚗 统一的仿真接口
- **UnifiedSimulator类**: 提供统一的API接口，支持不同场景
- **场景配置**: 通过参数配置不同的轨迹和障碍物
- **模块化设计**: 易于扩展新的场景类型

### 🎯 智能避障与轨迹回归
- **实时避障**: 基于DPCBF的安全约束优化
- **轨迹回归**: 避障后自动计算回到参考轨迹的路径
- **轨迹延长策略**: 在终点后延长参考轨迹，确保接近终点时仍有足够的参考信息

### 📊 统一的可视化系统
- **静态分析图**: 轨迹、速度、安全距离、控制输入分析
- **动态动画**: 实时展示车辆运动和避障过程
- **多场景支持**: 自动适配不同场景的可视化需求

## 支持的场景

### 1. 直线避障场景
- **轨迹**: 从(1,7.5)到(50,7.5)的直线路径
- **障碍物**: 4个动态障碍车辆，不同的运动模式
- **特点**: 高速直线行驶中的多车避障

### 2. 路口转弯场景
- **轨迹**: 1/4圆弧右转，从(0,0)到(15,15)
- **障碍物**: 3个车辆，包括对向左转车辆
- **特点**: 复杂路口环境下的转弯避障

## 快速开始

### 1. 安装依赖
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libosqp-dev libeigen3-dev libyaml-cpp-dev libgflags-dev
```

### 2. 编译项目
```bash
mkdir build && cd build
cmake ..
make -j4
```

### 3. 运行仿真
```bash
# 直线场景（保存CSV）
./unified_examples --scenario straight

# 路口场景（不保存CSV，适合在线运行）
./unified_examples --scenario intersection --save_csv=false

# 运行所有场景
./unified_examples --scenario both
```

### 4. 生成可视化
```bash
# 生成静态分析图和动画
python3 viz/unified_visualizer.py --scenario straight
python3 viz/unified_visualizer.py --scenario intersection
```

## 核心算法

### 1. 动态抛物线CBF (DPCBF)
- 基于抛物线边界的安全约束
- 实时计算安全距离和约束梯度
- 支持动态障碍物的预测

### 2. 轨迹回归算法
```cpp
// 计算回到参考轨迹的目标轨迹
auto return_spline = computeReturnTrajectory(current_state, ref_spline, duration);

// 计算未来控制量作为QP参考
Control future_control = computeFutureControl(return_spline, model, future_time);
```

### 3. 轨迹延长策略
- **核心思想**: 在原始参考轨迹终点后沿最后一段方向延长3秒距离
- **作用**: 确保接近终点时仍有足够的参考轨迹用于回归计算
- **重要**: 延长仅用于参考，车辆仍在原始终点停止

## 参数配置

### 机器人参数
- `radius`: 车辆半径 (0.3m)
- `L`: 轴距 (1.0m)
- `steer_max`: 最大转向角 (0.5 rad)
- `a_max`: 最大加速度 (5.0 m/s²)

### DPCBF参数
- `margin`: 安全边距 (2.2m)
- `k_lambda`: 收敛参数 (8.0)
- `k_mu`: 阻尼参数 (0.05)

### 仿真参数
- `dt`: 时间步长 (0.05s)
- `v_ref`: 参考速度 (6.0 m/s)
- `save_csv`: 是否保存CSV文件 (默认true)

## 使用示例

### 命令行参数
```bash
# 基本运行
./unified_examples --scenario straight

# 自定义参数
./unified_examples --scenario intersection --dt=0.02 --v_ref=4.0

# 在线运行（不保存CSV）
./unified_examples --scenario straight --save_csv=false

# 查看所有参数
./unified_examples --help
```

### 编程接口
```cpp
#include "include/unified_simulator.h"

// 创建仿真器
UnifiedSimulator simulator;

// 配置场景
std::vector<Waypoint> waypoints = {{0, 0, 0}, {10, 10, M_PI/4}};
std::vector<Obstacle> obstacles = {{5, 2, 1.0, 1.0, -1.0}};

ScenarioParams params;
params.type = ScenarioType::STRAIGHT_LINE;
params.output_filename = "output.csv";

// 运行仿真
simulator.runSimulation(waypoints, obstacles, params);
```

## 输出文件

### 仿真数据 (CSV格式)
- `output_dpcbf.csv`: 直线场景数据
- `intersection_output.csv`: 路口场景数据

包含字段：时间、位置、速度、控制输入、参考轨迹信息、障碍物状态、安全距离

### 可视化输出
- `*_analysis.png`: 静态分析图
- `*_animation.gif`: 动态动画

## 扩展新场景

1. **定义轨迹和障碍物**:
```cpp
std::vector<Waypoint> custom_waypoints = {
    {x1, y1, theta1}, {x2, y2, theta2}, ...
};

std::vector<Obstacle> custom_obstacles = {
    {ox, oy, r, vx, vy}, ...
};
```

2. **配置并运行**:
```cpp
ScenarioParams params;
params.type = ScenarioType::CUSTOM;
params.output_filename = FLAGS_save_csv ? "custom_output.csv" : "";

simulator.runSimulation(custom_waypoints, custom_obstacles, params);
```

## 故障排除

### 编译问题
- 确保安装了所有依赖库
- 检查CMake版本 >= 3.12

### 运行问题
- 检查轨迹点数量 >= 2
- 验证障碍物参数合理性

### 可视化问题
- 安装Python依赖: `pip install pandas matplotlib numpy`
- 确保CSV文件存在且格式正确

## 文件结构
```
cpp/
├── CMakeLists.txt
├── unified_examples.cpp        # 主程序入口
├── include/
│   ├── unified_simulator.h     # 统一仿真器接口
│   ├── bicycle_model.hpp       # 车辆动力学模型
│   ├── dpcbf.hpp              # DPCBF算法实现
│   ├── qp_controller.hpp       # QP控制器
│   └── SplineTrajectory.hpp    # 样条轨迹生成
├── src/
│   ├── unified_simulator.cpp   # 统一仿真器实现
│   └── gflags_config.cc       # 参数配置
└── viz/
    ├── unified_visualizer.py   # 统一可视化脚本
    └── plot.py                # 传统可视化脚本
```

## 技术特点

- **零依赖QP求解**: 使用OSQP-Eigen进行约束优化
- **实时性能**: 支持在线运行，可选择不保存CSV以提升性能
- **数值稳定**: 轨迹延长策略避免了边界条件问题
- **模块化设计**: 易于扩展和维护

---

**版本**: v2.0  
**最后更新**: 2024年10月