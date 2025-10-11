# 安全控制库 (safe_control) 代码逻辑分析

## 项目概述

`safe_control` 是一个用于机器人导航安全控制的Python库，实现了基于控制屏障函数(CBF)的各种安全关键控制器。该项目支持多种机器人动力学模型和控制器类型，包含完整的仿真、可视化和实验功能。

## 核心架构

### 1. 主要模块结构

```
Dynamic-Parabolic-CBF/
├── tracking.py              # 主要的跟踪控制器类
├── robots/                  # 机器人动力学模型
│   ├── robot.py            # 基础机器人基类
│   ├── unicycle2D.py       # 单轮模型
│   ├── dynamic_unicycle2D.py  # 动态单轮模型
│   └── ...
├── position_control/        # 位置控制器
│   ├── cbf_qp.py           # CBF-QP控制器
│   ├── mpc_cbf.py          # MPC-CBF控制器
│   └── ...
├── attitude_control/       # 姿态控制器
│   ├── gatekeeper_attitude.py # Gatekeeper安全过滤器
│   ├── simple_attitude.py     # 简单姿态控制
│   └── ...
├── dynamic_env/            # 动态环境处理
└── utils/                  # 工具函数
```

## 核心类分析

### 1. LocalTrackingController (tracking.py:34)

**主要功能**: 实现基于CBF的局部路径跟踪控制器

**关键方法**:
- `__init__()`: 初始化控制器，选择位置和姿态控制器
- `control_step()`: 执行单步控制计算
- `run_all_steps()`: 执行完整跟踪过程

**状态机逻辑**:
- `idle`: 空闲状态
- `track`: 路径跟踪状态
- `stop`: 停止状态
- `rotate`: 原地旋转状态

**控制流程**:
```python
1. 更新状态机
2. 检测障碍物
3. 计算标称控制输入
4. 更新CBF约束
5. 求解控制问题
6. 更新姿态控制器
7. 检查碰撞
8. 更新机器人状态
```

### 2. BaseRobot (robots/robot.py:28)

**主要功能**: 机器人基类，封装机器人的动力学、感知和可视化

**支持的机器人模型**:
- `SingleIntegrator2D`: 单积分器
- `DoubleIntegrator2D`: 双积分器
- `Unicycle2D`: 单轮模型
- `DynamicUnicycle2D`: 动态单轮模型
- `Quad2D/Quad3D`: 四旋翼模型
- `VTOL2D`: 垂直起降飞机

**关键功能**:
- 机器人动力学建模 (f(), g() 方法)
- 传感器模拟 (RGB-D相机)
- 安全区域计算
- 障碍物检测
- 可视化渲染

### 3. 控制器类型

#### 3.1 位置控制器 (position_control/)

**CBF-QP控制器** (cbf_qp.py:4)
- 基于二次规划的CBF控制器
- 支持相对度1和相对度2的CBF
- 可处理多个障碍物约束

**MPC-CBF控制器** (mpc_cbf.py)
- 结合模型预测控制和CBF
- 使用离散时间CBF约束
- 提供更长的预测视野

#### 3.2 姿态控制器 (attitude_control/)

**Gatekeeper控制器** (gatekeeper_attitude.py:21)
- 在标称控制器和备份控制器之间切换
- 保证无限时间的安全性
- 基于关键点可见性评估

## 核心算法原理

### 1. 控制屏障函数(CBF)

**基本概念**: 通过定义安全集合 $S = \{x \in \mathbb{R}^n : h(x) \geq 0\}$，确保系统状态始终保持在安全集合内。

**CBF条件**: 对于标称控制器 $u_{nom}$，求解优化问题：

$$
\min_{u} \|u - u_{nom}\|^2
$$

$$
\text{s.t. } \dot{h}(x,u) \geq -\alpha h(x)
$$

### 2. 动态障碍物处理

项目支持两种动态障碍物CBF方法：
- **C3BF**: 用于自行车模型的兼容性CBF
- **DPCBF**: 动态抛物CBF，处理动态障碍物

### 3. 多机器人协同

支持异构多机器人系统，每个机器人可以有不同的：
- 动力学模型
- 传感器配置
- 控制参数

## 使用示例

### 单机器人导航

```python
from safe_control.tracking import LocalTrackingController

# 机器人规格
robot_spec = {
    'model': 'DynamicUnicycle2D',
    'w_max': 0.5,
    'a_max': 0.5,
    'radius': 0.25
}

# 控制器类型
controller_type = {'pos': 'cbf_qp', 'att': 'velocity_tracking_yaw'}

# 初始化控制器
controller = LocalTrackingController(x_init, robot_spec, controller_type)

# 设置路径点
controller.set_waypoints(waypoints)

# 运行跟踪
controller.run_all_steps(tf=100)
```

### 多机器人场景

```python
# 机器人1
robot_spec_0 = {'model': 'DynamicUnicycle2D', 'robot_id': 0, ...}
controller_0 = LocalTrackingController(x_init, robot_spec_0, controller_type)

# 机器人2  
robot_spec_1 = {'model': 'DynamicUnicycle2D', 'robot_id': 1, ...}
controller_1 = LocalTrackingController(x_goal, robot_spec_1, controller_type)

# 协同运行
for _ in range(int(tf / dt)):
    controller_0.control_step()
    controller_1.control_step()
```

## 关键技术特性

### 1. 实时可视化
- 使用matplotlib的交互模式
- 实时显示机器人状态、传感器视野、安全区域
- 支持动画录制和导出

### 2. 传感器模拟
- RGB-D相机模型
- 有限视野和范围
- 动态障碍物检测

### 3. 安全保证
- 基于CBF的形式化安全保证
- 碰撞检测和避免
- 可见性约束处理

### 4. 可扩展性
- 模块化设计，易于添加新控制器
- 支持多种机器人动力学模型
- 灵活的配置系统

## 文件关键路径

- **主控制器**: `tracking.py:34` (LocalTrackingController类)
- **机器人基类**: `robots/robot.py:28` (BaseRobot类)
- **CBF-QP控制器**: `position_control/cbf_qp.py:4` (CBFQP类)
- **Gatekeeper控制器**: `attitude_control/gatekeeper_attitude.py:21` (GatekeeperAtt类)

## 依赖关系

项目依赖于以下主要库：
- `cvxpy`: 凸优化求解
- `numpy`: 数值计算
- `matplotlib`: 可视化
- `gurobipy`: 商业优化求解器
- `shapely`: 几何计算
- `do-mpc`: 模型预测控制

这个库为机器人安全导航提供了一个完整、可扩展的框架，特别适合研究基于CBF的安全控制方法。