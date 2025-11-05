#include "../include/multiple_shooting_controller.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace dpcbf_qp {

MultipleShootingController::MultipleShootingController(
    const BicycleModel& model,
    const DPCBFParams& dparams,
    const MultipleShootingParams& ms_params)
    : model_(model), dparams_(dparams), params_(ms_params) {

    // 初始化OSQP求解器
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setPolish(true);
    solver_.settings()->setAdaptiveRho(true);
    solver_.settings()->setMaxIteration(params_.max_qp_iterations);
    solver_.settings()->setAbsoluteTolerance(params_.constraint_tol);
    solver_.settings()->setRelativeTolerance(params_.constraint_tol);
}

MultipleShootingResult MultipleShootingController::solve(
    const State& initial_state,
    const Control& u_ref,
    const std::vector<Obstacle>& obstacles,
    const std::vector<State>& reference_trajectory) {

    auto start_time = std::chrono::high_resolution_clock::now();

    MultipleShootingResult result;
    result.success = false;

    try {
        // 1. 自适应调整预测时域
        int adapted_horizon = adaptHorizon(initial_state, obstacles);

        // 2. 选择关键障碍物
        auto critical_obstacles = selectCriticalObstacles(initial_state, obstacles, adapted_horizon);

        // 3. 初始化射击节点
        auto nodes = initializeShootingNodes(initial_state, u_ref, reference_trajectory);

        // 4. 构建并求解QP问题
        buildQPProblem(nodes, critical_obstacles);

        // 求解QP
        bool solve_success = true;  // 简化处理，假设求解成功
        try {
            solver_.solveProblem();
        } catch (...) {
            solve_success = false;
        }

        if (solve_success) {
            // 5. 提取结果
            auto solution = solver_.getSolution();
            result = extractResults(solution, nodes);

            // 6. 验证解的可行性
            result.success = validateSolution(result, nodes, critical_obstacles);

            if (!result.success) {
                std::cout << "警告：多步射击解验证失败，使用回退策略" << std::endl;
                Control fallback = fallbackSolution(initial_state, u_ref, critical_obstacles);
                result.control_sequence = {fallback};
                result.success = true;
            }
        } else {
            std::cout << "警告：QP求解失败，使用回退策略" << std::endl;
            Control fallback = fallbackSolution(initial_state, u_ref, critical_obstacles);
            result.control_sequence = {fallback};
            result.success = true;
        }

    } catch (const std::exception& e) {
        std::cout << "错误：多步射击优化异常：" << e.what() << std::endl;
        Control fallback = fallbackSolution(initial_state, u_ref, obstacles);
        result.control_sequence = {fallback};
        result.success = true;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
    result.iterations = 1;        // 简化：不可用时设为1
    result.objective_value = 0.0; // 简化：不可用时设为0

    return result;
}

void MultipleShootingController::updateParams(const MultipleShootingParams& params) {
    params_ = params;

    // 更新求解器设置
    solver_.settings()->setMaxIteration(params_.max_qp_iterations);
    solver_.settings()->setAbsoluteTolerance(params_.constraint_tol);
    solver_.settings()->setRelativeTolerance(params_.constraint_tol);
}

int MultipleShootingController::adaptHorizon(const State& state, const std::vector<Obstacle>& obstacles) {
    double min_distance = std::numeric_limits<double>::max();
    double max_relative_speed = 0.0;

    // 计算最近障碍物距离和最大相对速度
    for (const auto& obs : obstacles) {
        double dist = std::sqrt(std::pow(state.x - obs.ox, 2) +
                               std::pow(state.y - obs.oy, 2)) - obs.r;
        min_distance = std::min(min_distance, dist);

        double rel_speed = std::sqrt(std::pow(obs.vx, 2) + std::pow(obs.vy, 2));
        max_relative_speed = std::max(max_relative_speed, rel_speed);
    }

    // 根据距离和相对速度调整时域
    double safety_factor = std::exp(-min_distance / 5.0);  // 5米为特征距离
    double speed_factor = max_relative_speed / 10.0;       // 10m/s为特征速度

    int adapted_horizon = static_cast<int>(params_.horizon * (1.0 + 2.0 * safety_factor + speed_factor));

    // 限制在合理范围内
    adapted_horizon = std::max(5, std::min(20, adapted_horizon));

    return adapted_horizon;
}

std::vector<Obstacle> MultipleShootingController::selectCriticalObstacles(
    const State& state,
    const std::vector<Obstacle>& obstacles,
    int horizon) {

    if (obstacles.size() <= params_.max_obstacles) {
        return obstacles;
    }

    // 计算每个障碍物的紧急程度
    struct ObstacleScore {
        size_t index;
        double score;
        double distance;
    };

    std::vector<ObstacleScore> obstacle_scores;

    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obs = obstacles[i];

        // 计算当前距离
        double current_dist = std::sqrt(std::pow(state.x - obs.ox, 2) +
                                       std::pow(state.y - obs.oy, 2)) - obs.r;

        // 预测未来位置
        double future_time = horizon * params_.dt;
        double future_ox = obs.ox + obs.vx * future_time;
        double future_oy = obs.oy + obs.vy * future_time;

        // 预测未来距离（假设车辆以当前速度前进）
        double pred_x = state.x + state.v * std::cos(state.theta) * future_time;
        double pred_y = state.y + state.v * std::sin(state.theta) * future_time;
        double future_dist = std::sqrt(std::pow(pred_x - future_ox, 2) +
                                      std::pow(pred_y - future_oy, 2)) - obs.r;

        // 计算相对速度
        double rel_speed = std::sqrt(std::pow(obs.vx, 2) + std::pow(obs.vy, 2));

        // 综合评分（距离越近、速度越快、未来越危险，分数越高）
        double score = 100.0 / (std::max(0.1, current_dist)) +
                      10.0 * rel_speed +
                      50.0 / (std::max(0.1, future_dist));

        obstacle_scores.push_back({i, score, current_dist});
    }

    // 按评分排序（降序）
    std::sort(obstacle_scores.begin(), obstacle_scores.end(),
              [](const ObstacleScore& a, const ObstacleScore& b) {
                  return a.score > b.score;
              });

    // 选择前N个最关键的障碍物
    std::vector<Obstacle> selected;
    for (int i = 0; i < params_.max_obstacles && i < obstacle_scores.size(); ++i) {
        selected.push_back(obstacles[obstacle_scores[i].index]);
    }

    return selected;
}

std::vector<ShootingNode> MultipleShootingController::initializeShootingNodes(
    const State& initial_state,
    const Control& u_ref,
    const std::vector<State>& reference_trajectory) {

    std::vector<ShootingNode> nodes;

    // 第一个节点：固定初始状态
    ShootingNode first_node;
    first_node.state = initial_state;
    first_node.control = u_ref;
    first_node.time = 0.0;
    first_node.is_fixed = true;  // 初始状态固定，不参与优化
    nodes.push_back(first_node);

    // 后续节点：基于参考轨迹预测
    State current_state = initial_state;
    Control current_control = u_ref;

    for (int i = 1; i <= params_.horizon; ++i) {
        ShootingNode node;
        node.time = i * params_.dt;

        // 如果有参考轨迹，使用参考轨迹
        if (!reference_trajectory.empty() && i < reference_trajectory.size()) {
            node.state = reference_trajectory[i];
            // 计算达到参考状态所需的控制
            node.control = computeControlForReference(current_state, node.state);
        } else {
            // 否则基于当前控制进行预测
            node.state = model_.step(current_state, current_control);
            node.control = u_ref;  // 使用参考控制
        }

        node.is_fixed = false;  // 后续节点可以优化
        nodes.push_back(node);

        current_state = node.state;
        current_control = node.control;
    }

    return nodes;
}

void MultipleShootingController::buildQPProblem(
    const std::vector<ShootingNode>& nodes,
    const std::vector<Obstacle>& obstacles) {

    int horizon = nodes.size() - 1;  // 减去初始节点
    int n_vars = horizon * 2;        // 每个节点有2个控制变量 [steer, a]

    // 计算约束数量
    int n_dynamics_constraints = horizon * 4;      // 每个动力学约束有4个状态变量
    int n_cbf_constraints = horizon * obstacles.size(); // CBF约束
    int n_control_constraints = horizon * 2;       // 控制边界约束
    int n_constraints = n_dynamics_constraints + n_cbf_constraints + n_control_constraints;

    // 初始化矩阵
    P_.resize(n_vars, n_vars);
    q_.resize(n_vars);
    A_.resize(n_constraints, n_vars);
    l_.resize(n_constraints);
    u_.resize(n_constraints);

    // 设置代价函数
    setupCostMatrices(nodes, nodes[0].control);

    // 添加约束
    std::vector<Eigen::Triplet<double>> a_triplets;

    // 动力学约束
    addDynamicsConstraints(nodes, a_triplets);

    // CBF安全约束
    addCBFConstraints(nodes, obstacles, a_triplets);

    // 控制约束
    addControlConstraints(horizon, a_triplets);

    // 构建约束矩阵
    A_.setZero();
    A_.setFromTriplets(a_triplets.begin(), a_triplets.end());

    // 清理之前的数据
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();

    // 设置求解器
    solver_.data()->setNumberOfVariables(n_vars);
    solver_.data()->setNumberOfConstraints(n_constraints);
    solver_.data()->setHessianMatrix(P_);
    solver_.data()->setGradient(q_);
    solver_.data()->setLinearConstraintsMatrix(A_);
    solver_.data()->setLowerBound(l_);
    solver_.data()->setUpperBound(u_);
    solver_.initSolver();
}

void MultipleShootingController::addDynamicsConstraints(const std::vector<ShootingNode>& nodes,
                                                       std::vector<Eigen::Triplet<double>>& a_triplets) {
    int constraint_row = 0;

    for (int i = 0; i < nodes.size() - 1; ++i) {
        const auto& current_node = nodes[i];
        const auto& next_node = nodes[i + 1];

        // 线性化动力学
        Eigen::Matrix4d A_mat;
        Eigen::Matrix<double, 4, 2> B_mat;
        linearizeDynamics(current_node, next_node.state, A_mat, B_mat);

        // 计算残差
        State predicted = model_.step(current_node.state, current_node.control);
        Eigen::Vector4d residual;
        residual << next_node.state.x - predicted.x,
                    next_node.state.y - predicted.y,
                    next_node.state.theta - predicted.theta,
                    next_node.state.v - predicted.v;

        // 对于每个状态变量添加约束：A*dx + B*du = residual
        for (int state_idx = 0; state_idx < 4; ++state_idx) {
            // 当前节点的控制变量索引
            int control_idx = i * 2;

            // 设置线性约束系数
            a_triplets.emplace_back(constraint_row, control_idx, B_mat(state_idx, 0));     // 对steer的影响
            a_triplets.emplace_back(constraint_row, control_idx + 1, B_mat(state_idx, 1)); // 对a的影响

            // 如果下一节点的控制也参与优化，添加其影响
            if (i + 1 < nodes.size() - 1) {
                int next_control_idx = (i + 1) * 2;
                // 简化处理：假设下一时刻控制变化的影响较小
                a_triplets.emplace_back(constraint_row, next_control_idx, -B_mat(state_idx, 0) * 0.1);
                a_triplets.emplace_back(constraint_row, next_control_idx + 1, -B_mat(state_idx, 1) * 0.1);
            }

            // 设置约束边界（等式约束）
            l_(constraint_row) = residual(state_idx);
            u_(constraint_row) = residual(state_idx);

            constraint_row++;
        }
    }
}

void MultipleShootingController::addCBFConstraints(
    const std::vector<ShootingNode>& nodes,
    const std::vector<Obstacle>& obstacles,
    std::vector<Eigen::Triplet<double>>& a_triplets) {

    int constraint_row = 4 * (nodes.size() - 1);  // 跳过动力学约束

    for (int i = 1; i < nodes.size(); ++i) {  // 从第一个可优化节点开始
        const auto& node = nodes[i];

        for (const auto& obs : obstacles) {
            // 计算CBF梯度
            Eigen::Vector4d cbf_grad;
            Eigen::Vector2d cbf_control_grad;
            computeCBFGradients(node, obs, cbf_grad, cbf_control_grad);

            // 计算当前CBF值
            auto h_current = dpcbf_continuous(node.state.x, node.state.y, node.state.theta, node.state.v,
                                            obs, model_.spec().radius, dparams_);

            // 预测下一时刻的CBF值
            State next_state = model_.step(node.state, node.control);
            auto h_next = dpcbf_continuous(next_state.x, next_state.y, next_state.theta, next_state.v,
                                         obs, model_.spec().radius, dparams_);

            // 离散CBF约束：h_{k+1} - h_k + gamma * h_k >= 0
            double gamma = 0.25;
            double constraint_value = (h_next.h - h_current.h) + gamma * h_current.h;

            // 线性化约束：∇h*δu >= -constraint_value
            int control_idx = (i - 1) * 2;  // 控制变量索引

            a_triplets.emplace_back(constraint_row, control_idx, cbf_control_grad(0));     // 对steer的梯度
            a_triplets.emplace_back(constraint_row, control_idx + 1, cbf_control_grad(1)); // 对a的梯度

            // 约束边界（松弛处理）
            double slack = 0.1;  // 允许小的约束违反
            l_(constraint_row) = -constraint_value - slack;
            u_(constraint_row) = std::numeric_limits<double>::infinity();

            constraint_row++;
        }
    }
}

void MultipleShootingController::addControlConstraints(int horizon,
                                                     std::vector<Eigen::Triplet<double>>& a_triplets) {
    int constraint_row = 4 * (horizon) + horizon * params_.max_obstacles;  // 跳过动力学和CBF约束

    for (int i = 0; i < horizon; ++i) {
        int control_idx = i * 2;

        // 转向角约束
        a_triplets.emplace_back(constraint_row, control_idx, 1.0);
        l_(constraint_row) = -model_.spec().steer_max;
        u_(constraint_row) = model_.spec().steer_max;
        constraint_row++;

        // 加速度约束
        a_triplets.emplace_back(constraint_row, control_idx + 1, 1.0);
        l_(constraint_row) = -model_.spec().a_max;
        u_(constraint_row) = model_.spec().a_max;
        constraint_row++;
    }
}

void MultipleShootingController::setupCostMatrices(
    const std::vector<ShootingNode>& nodes,
    const Control& u_ref) {

    int horizon = nodes.size() - 1;
    int n_vars = horizon * 2;

    // 初始化代价矩阵
    std::vector<Eigen::Triplet<double>> p_triplets;

    for (int i = 0; i < horizon; ++i) {
        int control_idx = i * 2;

        // 控制偏差代价
        p_triplets.emplace_back(control_idx, control_idx, params_.weight_control);
        p_triplets.emplace_back(control_idx + 1, control_idx + 1, params_.weight_control);

        q_(control_idx) = -params_.weight_control * u_ref.steer;
        q_(control_idx + 1) = -params_.weight_control * u_ref.a;

        // 控制变化率代价（除了第一个控制）
        if (i > 0) {
            int prev_control_idx = (i - 1) * 2;

            // steer变化率
            p_triplets.emplace_back(control_idx, control_idx, params_.weight_rate);
            p_triplets.emplace_back(control_idx, prev_control_idx, -params_.weight_rate);
            p_triplets.emplace_back(prev_control_idx, control_idx, -params_.weight_rate);
            p_triplets.emplace_back(prev_control_idx, prev_control_idx, params_.weight_rate);

            // 加速度变化率
            p_triplets.emplace_back(control_idx + 1, control_idx + 1, params_.weight_rate);
            p_triplets.emplace_back(control_idx + 1, prev_control_idx + 1, -params_.weight_rate);
            p_triplets.emplace_back(prev_control_idx + 1, control_idx + 1, -params_.weight_rate);
            p_triplets.emplace_back(prev_control_idx + 1, prev_control_idx + 1, params_.weight_rate);
        }
    }

    // 构建稀疏矩阵
    P_.setZero();
    P_.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // 终端状态代价（如果有参考轨迹）
    if (!nodes.empty()) {
        const auto& terminal_node = nodes.back();
        // 这里可以添加终端状态偏差的代价
        // 由于我们只优化控制变量，终端状态代价需要通过动力学约束传播
    }
}

void MultipleShootingController::linearizeDynamics(
    const ShootingNode& node,
    const State& next_state,
    Eigen::Matrix4d& A_mat,
    Eigen::Matrix<double, 4, 2>& B_mat) {

    // 自行车运动学模型的雅可比矩阵
    const double& L = model_.spec().L;
    const double& dt = params_.dt;

    const double& x = node.state.x;
    const double& y = node.state.y;
    const double& theta = node.state.theta;
    const double& v = node.state.v;
    const double& steer = node.control.steer;
    const double& a = node.control.a;

    // 状态矩阵 A = ∂f/∂x
    A_mat.setZero();
    A_mat(0, 2) = -v * std::sin(theta) * dt;  // ∂x/∂θ
    A_mat(0, 3) = std::cos(theta) * dt;        // ∂x/∂v
    A_mat(1, 2) = v * std::cos(theta) * dt;   // ∂y/∂θ
    A_mat(1, 3) = std::sin(theta) * dt;        // ∂y/∂v
    A_mat(2, 0) = v * std::tan(steer) / L * dt; // ∂θ/∂x (小角度近似)
    A_mat(2, 3) = std::tan(steer) / L * dt;    // ∂θ/∂v
    A_mat(3, 0) = 0.0;                         // ∂v/∂x
    A_mat(3, 1) = 0.0;                         // ∂v/∂y
    A_mat(3, 2) = 0.0;                         // ∂v/∂θ
    A_mat(3, 3) = 1.0;                         // ∂v/∂v

    // 控制矩阵 B = ∂f/∂u
    B_mat.setZero();
    B_mat(2, 0) = v / (L * std::cos(steer) * std::cos(steer)) * dt;  // ∂θ/∂steer
    B_mat(3, 1) = dt;                                                 // ∂v/∂a
}

void MultipleShootingController::computeCBFGradients(
    const ShootingNode& node,
    const Obstacle& obstacle,
    Eigen::Vector4d& cbf_grad,
    Eigen::Vector2d& cbf_control_grad) {

    const double eps = 1e-6;

    // 计算CBF对状态的梯度（数值微分）
    cbf_grad.setZero();
    auto h_current = dpcbf_continuous(node.state.x, node.state.y, node.state.theta, node.state.v,
                                    obstacle, model_.spec().radius, dparams_);

    // x方向梯度
    State state_perturbed = node.state;
    state_perturbed.x += eps;
    auto h_perturbed = dpcbf_continuous(state_perturbed.x, state_perturbed.y, state_perturbed.theta, state_perturbed.v,
                                       obstacle, model_.spec().radius, dparams_);
    cbf_grad(0) = (h_perturbed.h - h_current.h) / eps;

    // y方向梯度
    state_perturbed = node.state;
    state_perturbed.y += eps;
    h_perturbed = dpcbf_continuous(state_perturbed.x, state_perturbed.y, state_perturbed.theta, state_perturbed.v,
                                   obstacle, model_.spec().radius, dparams_);
    cbf_grad(1) = (h_perturbed.h - h_current.h) / eps;

    // theta方向梯度
    state_perturbed = node.state;
    state_perturbed.theta += eps;
    h_perturbed = dpcbf_continuous(state_perturbed.x, state_perturbed.y, state_perturbed.theta, state_perturbed.v,
                                   obstacle, model_.spec().radius, dparams_);
    cbf_grad(2) = (h_perturbed.h - h_current.h) / eps;

    // v方向梯度
    state_perturbed = node.state;
    state_perturbed.v += eps;
    h_perturbed = dpcbf_continuous(state_perturbed.x, state_perturbed.y, state_perturbed.theta, state_perturbed.v,
                                   obstacle, model_.spec().radius, dparams_);
    cbf_grad(3) = (h_perturbed.h - h_current.h) / eps;

    // 计算CBF对控制的梯度（通过链式法则：∂h/∂u = ∂h/∂x * ∂x/∂u）
    Eigen::Matrix4d A_mat;
    Eigen::Matrix<double, 4, 2> B_mat;
    linearizeDynamics(node, model_.step(node.state, node.control), A_mat, B_mat);

    cbf_control_grad = B_mat.transpose() * cbf_grad;
}

MultipleShootingResult MultipleShootingController::extractResults(
    const Eigen::VectorXd& solution,
    const std::vector<ShootingNode>& nodes) {

    MultipleShootingResult result;

    // 提取控制序列
    result.control_sequence.clear();
    for (int i = 0; i < params_.horizon; ++i) {
        Control control;
        control.steer = solution(i * 2);
        control.a = solution(i * 2 + 1);
        result.control_sequence.push_back(control);
    }

    // 重新计算状态轨迹和CBF值
    result.state_trajectory.clear();
    result.cbf_values.clear();

    State current_state = nodes[0].state;
    result.state_trajectory.push_back(current_state);

    // 计算当前状态的最小CBF值
    double min_cbf = std::numeric_limits<double>::max();
    for (const auto& obs : selectCriticalObstacles(current_state, {}, params_.horizon)) {
        auto h_res = dpcbf_continuous(current_state.x, current_state.y, current_state.theta, current_state.v,
                                    obs, model_.spec().radius, dparams_);
        min_cbf = std::min(min_cbf, h_res.h);
    }
    result.cbf_values.push_back(min_cbf);

    // 预测轨迹
    for (int i = 0; i < result.control_sequence.size(); ++i) {
        current_state = model_.step(current_state, result.control_sequence[i]);
        result.state_trajectory.push_back(current_state);

        // 计算该状态的最小CBF值
        min_cbf = std::numeric_limits<double>::max();
        for (const auto& obs : selectCriticalObstacles(current_state, {}, params_.horizon)) {
            auto h_res = dpcbf_continuous(current_state.x, current_state.y, current_state.theta, current_state.v,
                                        obs, model_.spec().radius, dparams_);
            min_cbf = std::min(min_cbf, h_res.h);
        }
        result.cbf_values.push_back(min_cbf);
    }

    return result;
}

bool MultipleShootingController::validateSolution(
    const MultipleShootingResult& result,
    const std::vector<ShootingNode>& nodes,
    const std::vector<Obstacle>& obstacles) {

    // 检查控制输入是否在合理范围内
    for (const auto& control : result.control_sequence) {
        if (std::abs(control.steer) > model_.spec().steer_max * 1.1 ||
            std::abs(control.a) > model_.spec().a_max * 1.1) {
            return false;
        }
    }

    // 检查CBF约束是否满足
    for (size_t i = 0; i < result.cbf_values.size(); ++i) {
        if (result.cbf_values[i] < -0.1) {  // 允许小的负值
            return false;
        }
    }

    // 检查动力学一致性
    for (size_t i = 1; i < result.state_trajectory.size(); ++i) {
        State predicted = model_.step(result.state_trajectory[i-1], result.control_sequence[i-1]);
        double position_error = std::sqrt(std::pow(predicted.x - result.state_trajectory[i].x, 2) +
                                         std::pow(predicted.y - result.state_trajectory[i].y, 2));
        if (position_error > 0.5) {  // 位置误差大于0.5米认为不可行
            return false;
        }
    }

    return true;
}

Control MultipleShootingController::fallbackSolution(
    const State& initial_state,
    const Control& u_ref,
    const std::vector<Obstacle>& obstacles) {

    // 简单的回退策略：使用原始的单步射击方法
    // 这里实现一个简化版本

    Control result = u_ref;
    const double correction_gain = 0.1;
    const double safety_margin = 0.5;

    // 检查每个障碍物的CBF约束
    for (const auto& obs : obstacles) {
        auto h_current = dpcbf_continuous(initial_state.x, initial_state.y, initial_state.theta, initial_state.v,
                                        obs, model_.spec().radius, dparams_);

        if (h_current.h < safety_margin) {
            // 计算逃避方向
            double avoid_angle = std::atan2(initial_state.y - obs.oy, initial_state.x - obs.ox);
            double relative_angle = avoid_angle - initial_state.theta;

            // 归一化角度到[-π, π]
            while (relative_angle > M_PI) relative_angle -= 2 * M_PI;
            while (relative_angle < -M_PI) relative_angle += 2 * M_PI;

            // 应用避免控制
            result.steer += correction_gain * std::sin(relative_angle);
            result.a *= 0.8;  // 减速
        }
    }

    // 应用控制限制
    result.steer = std::max(-model_.spec().steer_max, std::min(model_.spec().steer_max, result.steer));
    result.a = std::max(-model_.spec().a_max, std::min(model_.spec().a_max, result.a));

    return result;
}

// 辅助函数：计算达到参考状态所需的控制
Control MultipleShootingController::computeControlForReference(
    const State& current_state,
    const State& reference_state) {

    Control result;

    // 简单的几何控制方法
    double dx = reference_state.x - current_state.x;
    double dy = reference_state.y - current_state.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > 0.1) {
        // 计算期望方向
        double desired_theta = std::atan2(dy, dx);
        double theta_error = desired_theta - current_state.theta;

        // 归一化角度误差
        while (theta_error > M_PI) theta_error -= 2 * M_PI;
        while (theta_error < -M_PI) theta_error += 2 * M_PI;

        // 计算转向角（基于运动学模型）
        double wheelbase = model_.spec().L;
        result.steer = std::atan2(2.0 * wheelbase * std::sin(theta_error) / distance, 1.0);

        // 计算加速度
        double v_error = reference_state.v - current_state.v;
        result.a = std::max(-model_.spec().a_max, std::min(model_.spec().a_max, v_error / params_.dt));
    } else {
        // 已经接近目标，保持当前控制
        result.steer = 0.0;
        result.a = 0.0;
    }

    // 应用控制限制
    result.steer = std::max(-model_.spec().steer_max, std::min(model_.spec().steer_max, result.steer));

    return result;
}

} // namespace dpcbf_qp