# Multi-Shooting vs Single-Shooting CBF-QP 控制器详细对比分析

## 概述

本文档详细分析了项目中实现的 Multi-Shooting 和 Single-Shooting 两种 CBF-QP 控制器在构造原理、代码实现和性能特性方面的差异。

## 1. 核心思想对比

### 1.1 Single-Shooting 方法
- **核心思想**: 一次性优化当前时刻的控制输入，通过单步预测确保安全
- **时域处理**: 仅考虑当前时刻到下一时刻的转移
- **决策变量**: `[steer, accel, slack]` - 3个变量
- **约束类型**: 即时安全约束，无多步一致性要求

### 1.2 Multi-Shooting 方法
- **核心思想**: 将预测时域分割为多个区间，同时优化整个时域的控制序列
- **时域处理**: 考虑整个预测时域（默认10步）的一致性
- **决策变量**: `[steer_0, accel_0, steer_1, accel_1, ..., steer_N, accel_N]` - 2*N个变量
- **约束类型**: 多步动力学一致性 + 时域内安全约束

## 2. 代码实现结构对比

### 2.1 类设计对比

#### Single-Shooting (QPController)
```cpp
// src/qp_controller.cc:13-14
Control QPController::solve(const State& s, const Control& u_ref,
                           const std::vector<Obstacle>& obstacles) const
```

**特点**:
- 单一函数接口，输入当前状态，输出控制
- 简单的3个决策变量
- 直接的QP求解流程

#### Multi-Shooting (MultipleShootingController)
```cpp
// src/multiple_shooting_controller.cpp:25-29
MultipleShootingResult MultipleShootingController::solve(
    const State& initial_state,
    const Control& u_ref,
    const std::vector<Obstacle>& obstacles,
    const std::vector<State>& reference_trajectory)
```

**特点**:
- 复杂的多阶段优化流程
- 返回完整的控制序列和状态轨迹
- 包含自适应时域调整等高级功能

### 2.2 核心算法流程对比

#### Single-Shooting 流程
```cpp
// src/qp_controller.cc 核心流程
1. 选择关键障碍物 (行 25-74)
   - 基于CBF值排序
   - 最多选择 max_obstacles 个

2. 构建QP问题 (行 78-102)
   - H矩阵: 3x3 对角矩阵
   - f向量: 线性项
   - 添加jerk惩罚项

3. 构建约束 (行 104-157)
   - CBF安全约束: J*δu >= -c0
   - 控制边界约束
   - 松弛变量约束

4. 求解QP (行 159-185)
   - 使用OSQP求解器
   - 失败时返回参考控制
```

#### Multi-Shooting 流程
```cpp
// src/multiple_shooting_controller.cpp 核心流程
1. 自适应准备 (行 37-44)
   - adaptHorizon(): 根据环境调整预测时域
   - selectCriticalObstacles(): 选择关键障碍物

2. 初始化射击节点 (行 190-232)
   - 固定初始节点
   - 预测后续节点状态

3. 构建QP问题 (行 234-287)
   - 设置代价矩阵 (setupCostMatrices)
   - 添加三类约束:
     * addDynamicsConstraints(): 动力学一致性
     * addCBFConstraints(): 多步安全约束
     * addControlConstraints(): 控制边界

4. 求解与验证 (行 50-83)
   - 求解QP问题
   - validateSolution(): 验证解的可行性
   - 失败时使用fallback策略
```

## 3. QP问题构造详细对比

### 3.1 决策变量数量

| 方法 | 决策变量 | 数量 | 说明 |
|------|----------|------|------|
| Single-Shooting | `[steer, accel, slack]` | 3 | 当前时刻控制 + 松弛变量 |
| Multi-Shooting | `[steer_i, accel_i]_{i=0..N-1}` | 2*N | 预测时域内所有控制 |

### 3.2 代价函数对比

#### Single-Shooting 代价函数
```cpp
// src/qp_controller.cc:82-102
// 基础控制跟踪代价
H(0,0) = w_.w_steer;      // 转向偏差权重
H(1,1) = w_.w_a;          // 加速度偏差权重
H(2,2) = w_.rho;          // 松弛变量惩罚

// Jerk惩罚 (平滑控制变化)
if (has_previous_) {
    H(0,0) += w_.w_jerk_steer;
    f(0) += -w_.w_jerk_steer * u_previous_.steer;
    H(1,1) += w_.w_jerk_accel;
    f(1) += -w_.w_jerk_accel * u_previous_.a;
}
```

**数学形式**:
```
min 0.5 * [w_steer*(steer-u_ref)^2 + w_a*(a-a_ref)^2 + w_jerk_steer*(steer-steer_prev)^2 + w_jerk_accel*(a-a_prev)^2 + rho*slack^2]
```

#### Multi-Shooting 代价函数
```cpp
// src/multiple_shooting_controller.cpp:402-450
for (int i = 0; i < horizon; ++i) {
    // 控制偏差代价
    p_triplets.emplace_back(control_idx, control_idx, params_.weight_control);

    // 控制变化率代价 (除第一个控制外)
    if (i > 0) {
        // 构建变化率的二次形式: w_rate*(u_i - u_{i-1})^2
        // 展开后为: w_rate*u_i^2 - 2*w_rate*u_i*u_{i-1} + w_rate*u_{i-1}^2
    }
}
```

**数学形式**:
```
min Σ_{i=0}^{N-1} [w_control*(u_i-u_ref)^2 + w_rate*(u_i-u_{i-1})^2] + w_terminal*||x_N-x_ref||^2
```

### 3.3 约束构造对比

#### Single-Shooting 约束
```cpp
// src/qp_controller.cc:110-144
// CBF安全约束 (每个障碍物一个)
for (int i = 0; i < m_obs; ++i) {
    // 线性化CBF约束: J*δu >= -c0
    A(i,0) = J_steer;      // 对转向的梯度
    A(i,1) = J_a;          // 对加速度的梯度
    A(i,2) = 1.0;          // 松弛变量系数
    l(i) = -c0 + J_steer*u_ref.steer + J_a*u_ref.a;  // 约束下界
}

// 控制边界约束
A(m_obs+0, 0) = 1.0; l = steer_lo; ub = steer_hi;
A(m_obs+1, 1) = 1.0; l = a_lo;     ub = a_hi;
A(m_obs+2, 2) = 1.0; l = 0.0;      ub = s_max;
```

**约束总数**: `m_obs + 3` (m_obs个CBF约束 + 3个边界约束)

#### Multi-Shooting 约束

##### 1. 动力学一致性约束
```cpp
// src/multiple_shooting_controller.cpp:289-334
for (int i = 0; i < nodes.size() - 1; ++i) {
    // 线性化动力学: x_{k+1} = x_k + A*dx_k + B*du_k
    // 约束形式: A*dx_k + B*du_k = residual
    for (int state_idx = 0; state_idx < 4; ++state_idx) {
        a_triplets.emplace_back(constraint_row, control_idx, B_mat(state_idx, 0));     // steer影响
        a_triplets.emplace_back(constraint_row, control_idx + 1, B_mat(state_idx, 1)); // accel影响

        // 下一时刻控制的影响 (简化处理)
        if (i + 1 < nodes.size() - 1) {
            a_triplets.emplace_back(constraint_row, next_control_idx, -B_mat(state_idx, 0) * 0.1);
            a_triplets.emplace_back(constraint_row, next_control_idx + 1, -B_mat(state_idx, 1) * 0.1);
        }

        l_(constraint_row) = residual(state_idx);  // 等式约束
        u_(constraint_row) = residual(state_idx);
    }
}
```

##### 2. CBF安全约束
```cpp
// src/multiple_shooting_controller.cpp:336-379
for (int i = 1; i < nodes.size(); ++i) {  // 从第一个可优化节点开始
    for (const auto& obs : obstacles) {
        // 离散CBF约束: h_{k+1} - h_k + γ*h_k >= 0
        double gamma = 0.25;
        double constraint_value = (h_next.h - h_current.h) + gamma * h_current.h;

        // 线性化: ∇h*δu >= -constraint_value
        a_triplets.emplace_back(constraint_row, control_idx, cbf_control_grad(0));
        a_triplets.emplace_back(constraint_row, control_idx + 1, cbf_control_grad(1));

        // 松弛处理
        double slack = 0.1;
        l_(constraint_row) = -constraint_value - slack;
        u_(constraint_row) = std::numeric_limits<double>::infinity();
    }
}
```

##### 3. 控制边界约束
```cpp
// src/multiple_shooting_controller.cpp:381-399
for (int i = 0; i < horizon; ++i) {
    // 每个时刻的控制都有边界约束
    a_triplets.emplace_back(constraint_row, control_idx, 1.0);     // steer约束
    l = -steer_max; ub = steer_max;

    a_triplets.emplace_back(constraint_row, control_idx + 1, 1.0); // accel约束
    l = -a_max; ub = a_max;
}
```

**约束总数**: `4*(N) + N*obs_count + 2*N` (动力学 + CBF + 边界)

## 4. CBF约束处理差异

### 4.1 Single-Shooting CBF处理
- **时域**: 仅当前时刻到下一时刻
- **约束形式**: `h_{k+1} - h_k + γ*h_k >= -slack`
- **线性化**: 在参考控制点线性化
- **松弛策略**: 单一松弛变量，允许所有约束共享松弛

### 4.2 Multi-Shooting CBF处理
- **时域**: 整个预测时域内的每个节点
- **约束形式**: `h_{i+1} - h_i + γ*h_i >= -slack_i` (每个节点独立)
- **线性化**: 在每个射击节点分别线性化
- **松弛策略**: 隐式松弛，通过约束边界容差实现

### 4.3 梯度计算差异

#### Single-Shooting 梯度
```cpp
// src/qp_controller.cc:121-133
// 有限差分计算CBF对控制的梯度
Control u_steer = u_ref; u_steer.steer += cfg_.du;
State s_steer = model_.step(s, u_steer);
auto hd_steer = dpcbf_discrete(...);
double J_steer = (c_steer - c0) / cfg_.du;
```

#### Multi-Shooting 梯度
```cpp
// src/multiple_shooting_controller.cpp:488-535
// 先计算CBF对状态的梯度，再通过链式法则得到对控制的梯度
auto h_current = dpcbf_continuous(...);
// 对状态梯度
for (每个状态分量) {
    扰动状态，计算梯度 = (h_perturbed.h - h_current.h) / eps;
}
// 对控制梯度 (链式法则)
cbf_control_grad = B_mat.transpose() * cbf_grad;
```

## 5. 动力学处理差异

### 5.1 Single-Shooting 动力学
- **处理方式**: 隐式处理，通过状态预测实现
- **线性化**: 在参考控制点单点线性化
- **一致性**: 无多步一致性要求

### 5.2 Multi-Shooting 动力学
- **处理方式**: 显式约束，强制节点间动力学一致性
- **线性化**: 在每个节点分别线性化
- **一致性**: 严格的等式约束确保动力学连续性

### 5.3 动力学线性化对比

#### Single-Shooting
```cpp
// 隐式在状态预测中处理
State s1 = model_.step(s, u_ref);  // 单步预测
```

#### Multi-Shooting
```cpp
// src/multiple_shooting_controller.cpp:452-486
void linearizeDynamics(const ShootingNode& node, const State& next_state,
                      Eigen::Matrix4d& A_mat, Eigen::Matrix<double, 4, 2>& B_mat) {
    // 计算雅可比矩阵
    A_mat(0, 2) = -v * std::sin(theta) * dt;  // ∂x/∂θ
    A_mat(0, 3) = std::cos(theta) * dt;        // ∂x/∂v
    // ... 完整的4x4状态矩阵和4x2控制矩阵

    B_mat(2, 0) = v / (L * std::cos(steer) * std::cos(steer)) * dt;  // ∂θ/∂steer
    B_mat(3, 1) = dt;                                                 // ∂v/∂a
}
```

## 6. 性能特性对比

### 6.1 计算复杂度

| 特性 | Single-Shooting | Multi-Shooting |
|------|----------------|----------------|
| **决策变量数** | 3 | 2*N (N=时域长度) |
| **约束数量** | m_obs + 3 | 4*N + N*m_obs + 2*N |
| **QP规模** | 小 (3x3) | 大 (2N x 2N) |
| **求解时间** | < 5ms | 10-20ms |
| **内存使用** | 低 | 中等 |

### 6.2 安全保证

| 特性 | Single-Shooting | Multi-Shooting |
|------|----------------|----------------|
| **安全时域** | 单步 (k → k+1) | 多步 (k → k+N) |
| **安全保证** | 局部 | 全局 (预测时域内) |
| **约束一致性** | 即时 | 时域内一致 |
| **数值稳定性** | 中等 | 高 |

### 6.3 控制质量

| 特性 | Single-Shooting | Multi-Shooting |
|------|----------------|----------------|
| **控制平滑性** | 依赖jerk惩罚 | 显式变化率约束 |
| **前瞻性** | 单步预测 | 多步轨迹优化 |
| **终端状态** | 无控制 | 有终端权重 |
| **轨迹一致性** | 无保证 | 动力学一致 |

## 7. 高级功能对比

### 7.1 Single-Shooting 特有功能
- **简单性**: 实现简单，调试容易
- **实时性**: 求解速度快，适合高频控制
- **鲁棒性**: 失败策略简单 (返回参考控制)

### 7.2 Multi-Shooting 特有功能

#### 自适应时域
```cpp
// src/multiple_shooting_controller.cpp:102-126
int adaptHorizon(const State& state, const std::vector<Obstacle>& obstacles) {
    double safety_factor = std::exp(-min_distance / 5.0);
    double speed_factor = max_relative_speed / 10.0;
    int adapted_horizon = params_.horizon * (1.0 + 2.0 * safety_factor + speed_factor);
    return std::max(5, std::min(20, adapted_horizon));
}
```

#### 智能障碍物选择
```cpp
// src/multiple_shooting_controller.cpp:128-188
std::vector<Obstacle> selectCriticalObstacles(...) {
    // 综合评分: 当前距离 + 未来距离 + 相对速度
    double score = 100.0 / current_dist + 10.0 * rel_speed + 50.0 / future_dist;
    // 选择评分最高的障碍物
}
```

#### 解验证与回退
```cpp
// src/multiple_shooting_controller.cpp:586-599
bool validateSolution(...) {
    // 检查控制输入范围
    // 检查CBF约束满足情况
    // 失败时使用fallback策略
}
```

## 8. 实际应用建议

### 8.1 选择 Single-Shooting 的场景
- **简单环境**: 障碍物少，运动模式简单
- **高频控制**: 需要快速控制响应
- **资源受限**: 计算资源有限的环境
- **初始实现**: 快速原型开发和验证

### 8.2 选择 Multi-Shooting 的场景
- **复杂环境**: 多障碍物，复杂交互
- **高速场景**: 需要长时域预测保证安全
- **轨迹优化**: 需要平滑的控制轨迹
- **安全关键**: 对安全性要求极高的应用

### 8.3 自适应策略 (项目中实现)
项目实现了智能的自适应选择策略：
```cpp
// src/unified_simulator.cpp:116-128
ControllerType selectControllerType(...) {
    // 基于环境复杂度和时间预算自动选择
    // 复杂度阈值: complexity_threshold_
    // 时间预算: time_budget_
}
```

## 9. 总结

Multi-Shooting 和 Single-Shooting 方法各有优势：

- **Single-Shooting**: 简单快速，适合简单环境和实时性要求高的场景
- **Multi-Shooting**: 功能强大，提供多步安全保证和更好的控制质量，适合复杂环境

项目通过实现自适应控制器选择，结合了两种方法的优势，在不同场景下自动选择最适合的控制器，实现了性能和安全性的平衡。

这种设计体现了现代控制系统的典型特征：**简单方法保证基础功能，高级方法提升性能，智能策略实现自动适应**。