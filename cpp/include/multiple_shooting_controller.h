#pragma once
#include "bicycle_model.h"
#include "dpcbf.h"
#include "SplineTrajectory.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <memory>
#include <cmath>

namespace dpcbf_qp {

/**
 * @brief 多步射击方法中的节点结构
 *
 * 每个节点包含一个预测步的状态、控制输入和时间信息。
 * 节点之间通过动力学约束连接。
 */
struct ShootingNode {
    State state;              ///< 该节点的机器人状态 [x, y, θ, v]
    Control control;          ///< 该节点的控制输入 [转向角, 加速度]
    double time;              ///< 该节点的时间
    bool is_fixed;            ///< 该节点状态/控制是否固定（不参与优化）

    ShootingNode() : time(0.0), is_fixed(false) {}
    ShootingNode(const State& s, const Control& c, double t, bool fixed = false)
        : state(s), control(c), time(t), is_fixed(fixed) {}
};

/**
 * @brief 多步射击控制器参数
 */
struct MultipleShootingParams {
    int horizon{10};              ///< 预测时域长度（步数）
    double dt{0.05};              ///< 每个射击间隔的时间步长
    double weight_control{12.0};   ///< 控制输入偏差权重
    double weight_rate{5.0};       ///< 控制变化率权重
    double weight_terminal{100.0}; ///< 终端状态权重
    double slack_penalty{20.0};    ///< 约束违反惩罚项
    int max_obstacles{3};          ///< 考虑的最大障碍物数量
    double constraint_tol{1e-4};   ///< 约束容差
    int max_qp_iterations{4000};   ///< QP求解器最大迭代次数
};

/**
 * @brief 多步射击优化结果
 */
struct MultipleShootingResult {
    std::vector<Control> control_sequence;    ///< 优化后的控制序列
    std::vector<State> state_trajectory;      ///< 预测的状态轨迹
    std::vector<double> cbf_values;           ///< 轨迹上的CBF值
    double solve_time{0.0};                   ///< 计算时间
    bool success{false};                      ///< 优化成功标志
    int iterations{0};                        ///< 求解器迭代次数
    double objective_value{0.0};             ///< 最终目标函数值
};

/**
 * @brief 基于CBF-QP的多步射击控制器
 *
 * 实现多步射击方法，将预测时域分割为多个区间，每个区间都有独立的控制变量。
 * 通过等式约束确保区间之间的动力学一致性。
 *
 * 主要特性：
 * - 多步预测提供更好的安全性保证
 * - 射击节点间的动力学一致性约束
 * - 每个预测步的CBF安全约束
 * - 二次规划 formulation 保证实时性能
 * - 基于环境复杂度的自适应时域
 */
class MultipleShootingController {
public:
    /**
     * @brief 多步射击控制器构造函数
     *
     * @param model 自行车模型引用，用于动力学积分
     * @param dparams DPCBF参数，用于安全约束计算
     * @param ms_params 多步射击特定参数
     */
    MultipleShootingController(const BicycleModel& model,
                             const DPCBFParams& dparams,
                             const MultipleShootingParams& ms_params);

    /**
     * @brief 求解多步射击优化问题
     *
     * @param initial_state 当前机器人状态
     * @param u_ref 参考控制输入（来自标称控制器）
     * @param obstacles 需要考虑安全的障碍物向量
     * @param reference_trajectory 参考轨迹（可选）
     * @return MultipleShootingResult 包含优化控制序列和元数据
     */
    MultipleShootingResult solve(const State& initial_state,
                               const Control& u_ref,
                               const std::vector<Obstacle>& obstacles,
                               const std::vector<State>& reference_trajectory = {});

    /**
     * @brief 更新控制器参数
     *
     * @param params 新的多步射击参数
     */
    void updateParams(const MultipleShootingParams& params);

    /**
     * @brief 获取当前控制器参数
     *
     * @return 当前的多步射击参数
     */
    const MultipleShootingParams& getParams() const { return params_; }

    /**
     * @brief 基于环境复杂度自适应预测时域
     *
     * @param state 当前机器人状态
     * @param obstacles 障碍物向量
     * @return 适应后的时域长度
     */
    int adaptHorizon(const State& state, const std::vector<Obstacle>& obstacles);

    /**
     * @brief 选择最关键的障碍物进行优化
     *
     * @param state 当前机器人状态
     * @param obstacles 环境中的所有障碍物
     * @param horizon 用于选择的预测时域
     * @return 选定的关键障碍物子集
     */
    std::vector<Obstacle> selectCriticalObstacles(
        const State& state,
        const std::vector<Obstacle>& obstacles,
        int horizon);

private:
    const BicycleModel& model_;           ///< 自行车模型引用
    DPCBFParams dparams_;                 ///< DPCBF参数
    MultipleShootingParams params_;       ///< 多步射击参数

    // QP求解器数据结构
    Eigen::SparseMatrix<double> P_;       ///< Hessian矩阵
    Eigen::VectorXd q_;                   ///< 线性代价向量
    Eigen::SparseMatrix<double> A_;       ///< 约束矩阵
    Eigen::VectorXd l_, u_;               ///< 约束上下界
    OsqpEigen::Solver solver_;            ///< OSQP求解器实例

    /**
     * @brief 初始化优化用的射击节点
     *
     * @param initial_state 预测的起始状态
     * @param u_ref 参考控制输入
     * @param reference_trajectory 可选的参考轨迹
     * @return 初始化的射击节点向量
     */
    std::vector<ShootingNode> initializeShootingNodes(
        const State& initial_state,
        const Control& u_ref,
        const std::vector<State>& reference_trajectory);

    /**
     * @brief 构建二次规划问题
     *
     * @param nodes 当前优化的射击节点
     * @param obstacles 需要考虑安全约束的障碍物
     */
    void buildQPProblem(const std::vector<ShootingNode>& nodes,
                       const std::vector<Obstacle>& obstacles);

    /**
     * @brief 添加动力学一致性约束
     *
     * @param nodes 需要用动力学约束连接的射击节点
     * @param a_triplets 用于构建约束矩阵的三元组
     */
    void addDynamicsConstraints(const std::vector<ShootingNode>& nodes,
                               std::vector<Eigen::Triplet<double>>& a_triplets);

    /**
     * @brief 添加CBF安全约束
     *
     * @param nodes 强制CBF约束的射击节点
     * @param obstacles 用于安全约束的障碍物
     * @param a_triplets 用于构建约束矩阵的三元组
     */
    void addCBFConstraints(const std::vector<ShootingNode>& nodes,
                          const std::vector<Obstacle>& obstacles,
                          std::vector<Eigen::Triplet<double>>& a_triplets);

    /**
     * @brief 添加控制输入边界和变化率约束
     *
     * @param horizon 约束边界的预测时域
     * @param a_triplets 用于构建约束矩阵的三元组
     */
    void addControlConstraints(int horizon,
                              std::vector<Eigen::Triplet<double>>& a_triplets);

    /**
     * @brief 设置QP代价矩阵（目标函数）
     *
     * @param nodes 用于代价计算的射击节点
     * @param u_ref 参考控制输入
     */
    void setupCostMatrices(const std::vector<ShootingNode>& nodes,
                          const Control& u_ref);

    /**
     * @brief 在当前节点周围线性化动力学
     *
     * @param node 当前射击节点
     * @param next_state 预测的下一个状态
     * @param A_mat 状态矩阵（线性化结果）
     * @param B_mat 控制矩阵（线性化结果）
     */
    void linearizeDynamics(const ShootingNode& node,
                          const State& next_state,
                          Eigen::Matrix4d& A_mat,
                          Eigen::Matrix<double, 4, 2>& B_mat);

    /**
     * @brief 计算线性化的CBF约束系数
     *
     * @param node 当前射击节点
     * @param obstacle CBF约束的障碍物
     * @param cbf_grad CBF对状态的梯度
     * @param cbf_control_grad CBF对控制的梯度
     */
    void computeCBFGradients(const ShootingNode& node,
                            const Obstacle& obstacle,
                            Eigen::Vector4d& cbf_grad,
                            Eigen::Vector2d& cbf_control_grad);

    /**
     * @brief 从QP解中提取优化结果
     *
     * @param solution QP解向量
     * @param nodes 原始射击节点
     * @return MultipleShootingResult 包含提取的轨迹和控制
     */
    MultipleShootingResult extractResults(const Eigen::VectorXd& solution,
                                        const std::vector<ShootingNode>& nodes);

    /**
     * @brief 验证解的可行性
     *
     * @param result 需要验证的优化结果
     * @param nodes 用于验证的射击节点
     * @param obstacles 用于安全验证的障碍物
     * @return 如果解可行且安全则返回true
     */
    bool validateSolution(const MultipleShootingResult& result,
                         const std::vector<ShootingNode>& nodes,
                         const std::vector<Obstacle>& obstacles);

    /**
     * @brief 如果多步射击失败则回退到单步射击
     *
     * @param initial_state 当前机器人状态
     * @param u_ref 参考控制输入
     * @param obstacles 用于安全的障碍物
     * @return 简化的控制解
     */
    Control fallbackSolution(const State& initial_state,
                           const Control& u_ref,
                           const std::vector<Obstacle>& obstacles);

    /**
     * @brief 计算达到参考状态所需的控制
     *
     * @param current_state 当前状态
     * @param reference_state 参考状态
     * @return 控制输入
     */
    Control computeControlForReference(const State& current_state,
                                     const State& reference_state);
};

} // namespace dpcbf_qp