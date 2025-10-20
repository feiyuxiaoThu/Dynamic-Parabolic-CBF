# DPCBF代码优化策略计划

## 优化目标
1. 提高障碍物选择的智能化程度
2. 实现DPCBF参数的动态调整
3. 提升系统在复杂动态环境中的性能

## 优化计划（分阶段实施）

### 阶段1：基础设施准备 (第1周)
**任务列表：**
- [x] 创建新的配置结构体文件 `adaptive_config.h`
- [x] 实现基础的数据结构和类型定义
- [x] 为现有代码添加版本控制标记，确保可回滚

**具体实现：**
```cpp
// adaptive_config.h
#pragma once
#include <vector>
#include <array>

namespace dpcbf_qp {

// 障碍物选择配置
struct ObstacleSelectionParams {
    double base_h_threshold{0.2};           // 基础h值阈值
    double max_obstacles{3};                // 最大障碍物数量
    double min_consideration_distance{30.0}; // 考虑距离范围
    double max_ttc_consideration{8.0};      // 最大TTC考虑时间
    double speed_sensitivity{0.05};         // 速度敏感度
};

// 参数自适应配置  
struct ParameterAdaptationParams {
    double k_lambda_base{0.1};              // 基础k_lambda
    double k_mu_base{0.5};                  // 基础k_mu
    double speed_adaptation_factor{0.1};    // 速度适应因子
    double ttc_adaptation_factor{0.5};      // TTC适应因子
    double curvature_adaptation_factor{0.2}; // 曲率适应因子
};

// 障碍物评分结构
struct ObstacleScore {
    double h_value;
    double ttc;           // 碰撞时间
    double distance;      // 距离
    double path_relevance; // 路径相关性
    int obstacle_index;
    
    double combined_score() const {
        // 综合评分：结合DPCBF值、碰撞时间和路径相关性
        double ttc_factor = (ttc > 1e-6) ? (1.0 / ttc) : 1e6;
        return h_value + 0.3 * ttc_factor + 0.2 * (1.0 / (path_relevance + 1e-6));
    }
};

} // namespace dpcbf_qp
```

### 阶段2：智能障碍物选择实现 (第2周)
**任务列表：**
- [x] 实现时空结合的障碍物评分算法
- [x] 开发自适应阈值计算函数
- [x] 集成到QP控制器中
- [x] 单元测试验证

**具体实现：**
```cpp
// 在qp_controller.cc中
private:
    ObstacleSelectionParams obs_select_params_;
    
    double computeAdaptiveThreshold(const State& s) const;
    std::vector<ObstacleScore> computeObstacleScores(
        const State& s, 
        const std::vector<Obstacle>& obstacles) const;
    std::vector<int> selectObstacles(const State& s, 
                                   const std::vector<Obstacle>& obstacles) const;

// 实现函数
double QPController::computeAdaptiveThreshold(const State& s) const {
    // 基于车辆当前速度调整阈值
    double speed_factor = 1.0 + obs_select_params_.speed_sensitivity * s.v;
    return obs_select_params_.base_h_threshold * speed_factor;
}

std::vector<ObstacleScore> QPController::computeObstacleScores(
    const State& s, 
    const std::vector<Obstacle>& obstacles) const {
    
    std::vector<ObstacleScore> scores;
    scores.reserve(obstacles.size());
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        auto hres = dpcbf_continuous(s.x, s.y, s.theta, s.v, obstacles[i], 
                                   model_.spec().radius, dparams_);
        
        ObstacleScore score;
        score.h_value = hres.h;
        score.obstacle_index = static_cast<int>(i);
        
        // 计算碰撞时间TTC
        double px = obstacles[i].ox - s.x;
        double py = obstacles[i].oy - s.y;
        double vx_rel = obstacles[i].vx - s.v * std::cos(s.theta);
        double vy_rel = obstacles[i].vy - s.v * std::sin(s.theta);
        
        double pr = std::sqrt(px*px + py*py);
        double vxl, vyl;
        rotateToLOS(px, py, vx_rel, vy_rel, vxl, vyl);
        
        score.distance = pr;
        score.ttc = (vxl < -1e-6) ? (-pr / vxl) : 1e6;  // 只有接近时计算TTC
        
        // 计算路径相关性（简化版：距离车辆路径的横向距离）
        double dx = obstacles[i].ox - s.x;
        double dy = obstacles[i].oy - s.y;
        double robot_heading_x = std::cos(s.theta);
        double robot_heading_y = std::sin(s.theta);
        // 计算障碍物在车辆前进方向上的投影距离
        double proj = dx * robot_heading_x + dy * robot_heading_y;
        // 横向距离（偏离车辆路径的距离）
        double lateral_dist = std::abs(dx * (-robot_heading_y) + dy * robot_heading_x);
        score.path_relevance = lateral_dist;
        
        scores.push_back(score);
    }
    
    return scores;
}

std::vector<int> QPController::selectObstacles(
    const State& s, 
    const std::vector<Obstacle>& obstacles) const {
    
    auto scores = computeObstacleScores(s, obstacles);
    
    // 按综合评分排序
    std::sort(scores.begin(), scores.end(), 
              [](const auto& a, const auto& b) { 
                  return a.combined_score() < b.combined_score(); 
              });
    
    // 选择最危险的障碍物
    std::vector<int> selected;
    selected.reserve(obs_select_params_.max_obstacles);
    
    for (const auto& score : scores) {
        if (selected.size() >= obs_select_params_.max_obstacles) break;
        
        // 只选择在考虑范围内的障碍物
        if (score.distance <= obs_select_params_.min_consideration_distance && 
            score.ttc <= obs_select_params_.max_ttc_consideration) {
            selected.push_back(score.obstacle_index);
        }
    }
    
    return selected;
}
```

### 阶段3：参数自适应机制实现 (第3周) 
**任务列表：**
- [x] 实现参数自适应计算函数
- [x] 修改DPCBF函数以支持自适应参数
- [x] 集成到QP控制器的约束构建中
- [x] 验证自适应参数的正确性

**具体实现：**
```cpp
// 在dpcbf.h中添加
struct AdaptiveDPCBFParams {
    double k_lambda_adaptive;
    double k_mu_adaptive;
};

AdaptiveDPCBFParams computeAdaptiveParams(
    const State& s, 
    const Obstacle& obs,
    double reference_curvature,
    double robot_speed,
    const DPCBFParams& base_params
) const;

// 修改DPCBF函数
DPCBFResult dpcbf_continuous_adaptive(
    double X, double Y, double theta, double v,
    const Obstacle& obs, 
    double robot_radius,
    const DPCBFParams& base_params,
    double reference_curvature = 0.0
);

// 在QP控制器中使用
// 重构约束构建过程以使用自适应参数
```

### 阶段4：系统集成与测试 (第4周)
**任务列表：**
- [x] 将新功能集成到统一仿真器中
- [x] 开发配置文件或命令行参数接口
- [x] 完整系统功能测试
- [x] 性能基准测试

**实施要点：**
```cpp
// 统一仿真器中的自适应参数使用
class UnifiedSimulator {
private:
    // 确保参考轨迹曲率信息传递到DPCBF计算中
    double computeTrajectoryCurvature(
        const SplineTrajectory::QuinticSpline3D& spline, 
        double t) const;
    
    // 在仿真循环中计算并传递曲率信息
    void runSimulationStep(/* ... */);
};
```

### 阶段5：性能优化与调试 (第5周)
**任务列表：**
- [x] 性能分析与优化
- [x] 边界情况处理完善
- [x] 参数调优工具开发
- [x] 回归测试

## 实施过程中的关键注意事项

### 1. 保持向后兼容性
```cpp
// 提供向后兼容的接口
DPCBFResult dpcbf_continuous_legacy(/* 原有参数 */) {
    // 保持原有行为
    return dpcbf_continuous_adaptive(/* 调用新函数，使用默认值 */);
}
```

### 2. 逐步切换策略
```cpp
// 通过配置选项控制是否启用新功能
struct FeaturesConfig {
    bool enable_adaptive_obstacle_selection{false};
    bool enable_parameter_adaptation{false};
    // ... 其他功能开关
};
```

### 3. 全面的测试覆盖
- 单元测试：验证每个新函数的正确性
- 集成测试：验证组件间协作
- 回归测试：确保原有功能不受影响
- 边界测试：验证极端情况下的行为

### 4. 文档更新
- 更新API文档
- 添加使用示例
- 编写配置说明

## 预期成果

完成优化后，系统将具备：
1. **智能障碍物选择**：基于时空多维度评估选择最相关的障碍物
2. **参数自适应**：根据驾驶情境动态调整安全参数
3. **性能提升**：在保持安全性的同时提高避障效率
4. **可扩展性**：为未来更多智能功能提供基础

这个分阶段的优化计划将确保每次更新都经过充分测试，降低系统风险，同时逐步提升DPCBF系统的智能性。