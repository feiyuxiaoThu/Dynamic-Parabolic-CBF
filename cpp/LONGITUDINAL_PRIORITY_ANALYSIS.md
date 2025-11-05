# 纵向优先避障控制策略分析

## 问题背景

在自动驾驶场景中，希望优先采用纵向避让策略（减速/停车），而仅在必要时进行横向绕行。这种策略更符合：
- 人类驾驶员的自然行为
- 交通规则和道路约束
- 乘客舒适性要求
- 道路空间的有效利用

## 当前DPCBF避障行为分析

### 现有DPCBF函数
```cpp
// src/dpcbf.cc:93
res.h = vxl + lam * (vyl*vyl) + mu;
```

**组件分析**：
- `vxl`: 径向相对速度（正值表示远离障碍物）
- `lam * vyl²`: 横向速度的二次惩罚项
- `mu`: 基于距离的安全边界

**存在问题**：
1. 横向惩罚系数 `lam` 随距离增大而减小，远距离时横向运动约束较弱
2. 对称的横向惩罚，没有区分左右绕行的优先级
3. 缺乏对车辆朝向和道路结构的考虑

## 纵向优先避障优化策略

### 策略1: 非对称DPCBF函数设计

#### 1.1 引入方向性权重
```cpp
// 修改后的DPCBF函数
res.h = vxl + lam_left * vyl_left² + lam_right * vyl_right² + mu;

// 其中：
// - vyl_left: 向左的切向速度分量
// - vyl_right: 向右的切向速度分量
// - lam_left, lam_right: 非对称的横向惩罚系数
```

#### 1.2 基于道路几何的方向偏好
```cpp
// 根据车辆在道路中的位置调整横向权重
if (is_in_left_lane) {
    lam_right = lam * 0.5;  // 向右绕行更容易（避让对向来车）
    lam_left = lam * 2.0;   // 向左绕行更困难（可能压线）
} else {
    lam_left = lam * 0.5;   // 向左绕行更容易（避让同向来车）
    lam_right = lam * 2.0;  // 向右绕行更困难（可能出道路）
}
```

### 策略2: 速度依赖的动态权重

#### 2.1 基于相对速度的自适应策略
```cpp
// 根据相对速度调整横向惩罚
double relative_speed_factor = std::abs(vxl) / (vr + eps);
double lateral_penalty_boost = 1.0 + 5.0 * relative_speed_factor;

lam = base_lam * lateral_penalty_boost;

// 高速接近时强烈抑制横向运动
// 低速或远离时允许适度横向调整
```

#### 2.2 纵向优先的分级响应
```cpp
// 三级避障响应机制
if (vxl < -high_speed_threshold) {
    // 高速接近：强制纵向减速
    mu *= 3.0;           // 增强安全边界
    lam *= 5.0;          // 强烈抑制横向运动
} else if (vxl < -low_speed_threshold) {
    // 中速接近：平衡纵向和横向
    mu *= 1.5;           // 适度增强安全边界
    lam *= 2.0;          // 适度抑制横向运动
} else {
    // 低速或远离：允许横向调整
    // 使用基准参数
}
```

### 策略3: 代价函数权重调整

#### 3.1 增强转向惩罚
```cpp
// 在QP代价函数中增强对转向的惩罚
struct LongitudinalPriorityWeights {
    double w_steer{50.0};        // 大幅增加转向权重（原12.0）
    double w_a{12.0};            // 保持加速度权重
    double w_jerk_steer{20.0};   // 增强转向变化率惩罚（原5.0）
    double w_jerk_accel{200.0};  // 保持加速度变化率惩罚
    double w_speed_matching{30.0}; // 新增：速度匹配权重
};
```

#### 3.2 横向位置偏移惩罚
```cpp
// 添加对横向偏离参考轨迹的惩罚
double lateral_deviation_weight = 100.0;  // 横向偏离惩罚
double longitudinal_deviation_weight = 10.0; // 纵向偏离惩罚

// 在代价函数中添加
cost += lateral_deviation_weight * (y_current - y_reference)² +
        longitudinal_deviation_weight * (x_current - x_reference)²;
```

### 策略4: 非对称约束设计

#### 4.1 差异化横向约束
```cpp
// 为左右转向设置不同的约束边界
double max_left_steer = base_max_steer * 0.6;   // 限制左转幅度
double max_right_steer = base_max_steer * 1.0;  // 允许正常右转

// 根据障碍物位置动态调整
if (obstacle_is_to_the_left) {
    max_left_steer = base_max_steer * 0.3;  // 障碍物在左时严格限制左转
    max_right_steer = base_max_steer * 0.8; // 允许适度右转避让
}
```

#### 4.2 速度依赖的转向约束
```cpp
// 根据速度限制转向幅度
double speed_factor = v / v_max;
double dynamic_steer_limit = max_steer * (1.0 - 0.7 * speed_factor);

// 高速时严格限制转向，低速时允许更大转向
```

### 策略5: 分层避障决策

#### 5.1 决策树结构
```cpp
enum class AvoidanceStrategy {
    LONGITUDINAL_ONLY,    // 仅纵向避让（减速/停车）
    LONGITUDINAL_PRIMARY, // 纵向为主，横向为辅
    BALANCED,            // 纵横向平衡
    LATERAL_PRIMARY      // 横向为主（紧急情况）
};

AvoidanceStrategy selectStrategy(const State& state, const Obstacle& obs) {
    double distance = computeDistance(state, obs);
    double relative_speed = computeRelativeSpeed(state, obs);
    double ttc = distance / std::max(relative_speed, 0.1);

    if (ttc > 5.0) {
        return LONGITUDINAL_ONLY;      // 充足时间：仅减速
    } else if (ttc > 2.0) {
        return LONGITUDINAL_PRIMARY;   // 中等时间：纵向优先
    } else if (ttc > 1.0) {
        return BALANCED;              // 短时间：平衡策略
    } else {
        return LATERAL_PRIMARY;       // 紧急：允许横向规避
    }
}
```

#### 5.2 策略参数配置
```cpp
struct StrategyParams {
    AvoidanceStrategy strategy;
    double lateral_weight_multiplier;
    double longitudinal_weight_multiplier;
    double max_steer_factor;
    double cbf_aggressiveness;
};

// 根据选择的策略调整QP参数
void configureQPForStrategy(QPWeights& weights, DPCBFParams& cbf_params,
                           const StrategyParams& strategy) {
    weights.w_steer *= strategy.lateral_weight_multiplier;
    weights.w_jerk_steer *= strategy.lateral_weight_multiplier;
    cbf_params.k_lambda *= strategy.cbf_aggressiveness;
    // ... 其他参数调整
}
```

## 实现方案

### 方案1: 修改DPCBF函数（推荐）

**优点**：
- 直接在安全约束层面实现纵向优先
- 保持系统架构不变
- 效果直接明显

**实现步骤**：
1. 修改 `dpcbf.cc` 中的DPCBF函数
2. 添加方向性参数到 `DPCBFParams` 结构
3. 根据障碍物位置动态调整横向权重

### 方案2: 增强QP代价函数

**优点**：
- 不改变安全约束的理论基础
- 通过优化引导实现行为偏好
- 易于调参和验证

**实现步骤**：
1. 修改 `QPWeights` 结构
2. 在 `qp_controller.cc` 中更新代价函数
3. 添加横向偏离惩罚项

### 方案3: 分层决策+QP切换

**优点**：
- 最灵活的控制策略
- 可以实现复杂的行为逻辑
- 便于集成更多上下文信息

**实现步骤**：
1. 实现策略选择器
2. 为每种策略配置参数集
3. 在 `unified_simulator.cpp` 中集成策略切换

## 参数调优建议

### 关键参数范围
```cpp
// 纵向优先的推荐参数
struct LongitudinalPriorityConfig {
    // DPCBF参数
    double k_lambda_longitudinal{0.2};    // 增加横向惩罚
    double k_mu_longitudinal{0.8};        // 增强安全边界
    double margin_longitudinal{1.2};      // 增大安全系数

    // QP权重参数
    double w_steer_priority{30.0};        // 转向权重（原12.0）
    double w_jerk_steer_priority{15.0};   // 转向变化率权重（原5.0）

    // 约束参数
    double steer_limit_factor{0.6};       // 转向幅度限制系数
    double lateral_penalty_boost{3.0};    // 横向运动增强惩罚
};
```

### 调参流程
1. **基础调参**：确保安全性和稳定性
2. **行为调参**：根据仿真数据调整纵向/横向平衡
3. **边界测试**：验证极端场景下的行为
4. **实车标定**：根据实际车辆特性微调

## 验证指标

### 定量指标
- **横向偏移量**：避障过程中的最大横向位移
- **纵向减速幅度**：避障过程中的减速度分布
- **避障成功率**：成功避障的比例
- **轨迹平滑度**：控制输入的变化率

### 定性指标
- **行为自然度**：是否符合人类驾驶习惯
- **乘客舒适度**：避障过程的平顺性
- **道路规则遵守**：是否保持在车道内

## 总结

实现纵向优先避障需要在多个层面进行协同优化：

1. **约束层面**：修改DPCBF函数，增强对横向运动的约束
2. **优化层面**：调整QP代价函数，引导纵向避让行为
3. **决策层面**：实现分层策略，根据场景选择合适的控制模式
4. **参数层面**：精心设计参数，实现期望的行为特性

建议从**方案1（修改DPCBF函数）**开始实现，因为这种方法直接作用于安全约束，效果最明显，同时保持了系统的理论一致性。