#pragma once
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <memory>
#include <chrono>

namespace dpcbf_qp {

/**
 * @brief QP求解器配置参数
 */
struct QPSolverConfig {
    double absolute_tolerance{1e-6};      ///< 绝对容差
    double relative_tolerance{1e-6};      ///< 相对容差
    double primal_tolerance{1e-6};        ///< 原始容差
    double dual_tolerance{1e-6};          ///< 对偶容差
    int max_iterations{4000};             ///< 最大迭代次数
    bool verbose{false};                  ///< 是否显示详细信息
    bool warm_start{true};                ///< 是否使用热启动
    bool polish_solution{true};           ///< 是否优化解
    bool adaptive_rho{true};              ///< 是否自适应调整rho参数
    double time_limit{0.1};               ///< 求解时间限制（秒）
    int check_interval{100};              ///< 检查间隔
};

/**
 * @brief QP求解结果
 */
struct QPSolution {
    Eigen::VectorXd solution;             ///< 最优解向量
    double objective_value{0.0};          ///< 目标函数值
    int iterations{0};                    ///< 迭代次数
    double solve_time{0.0};               ///< 求解时间
    bool success{false};                  ///< 求解成功标志
    std::string status_message;           ///< 状态信息
    double primal_residual{0.0};          ///< 原始残差
    double dual_residual{0.0};            ///< 对偶残差
};

/**
 * @brief 大规模QP求解器封装类
 *
 * 提供高性能的二次规划求解功能，专门针对实时控制应用优化。
 * 支持稀疏矩阵、热启动、时间限制等高级特性。
 *
 * 主要特性：
 * - 稀疏矩阵支持，提高大规模问题求解效率
 * - 热启动功能，加速相似问题的求解
 * - 时间监控，确保实时性要求
 * - 解的验证和后处理
 * - 自适应参数调整
 */
class AdvancedQPSolver {
public:
    /**
     * @brief 构造函数
     *
     * @param config 求解器配置参数
     */
    explicit AdvancedQPSolver(const QPSolverConfig& config = QPSolverConfig{});

    /**
     * @brief 析构函数
     */
    ~AdvancedQPSolver();

    /**
     * @brief 设置QP问题数据
     *
     * @param H Hessian矩阵（稀疏）
     * @param f 线性项向量
     * @param A 约束矩阵（稀疏）
     * @param lower_bound 约束下界
     * @param upper_bound 约束上界
     * @return 是否设置成功
     */
    bool setupProblem(const Eigen::SparseMatrix<double>& H,
                     const Eigen::VectorXd& f,
                     const Eigen::SparseMatrix<double>& A,
                     const Eigen::VectorXd& lower_bound,
                     const Eigen::VectorXd& upper_bound);

    /**
     * @brief 求解QP问题
     *
     * @param warm_start 是否使用热启动
     * @return QP求解结果
     */
    QPSolution solve(bool warm_start = true);

    /**
     * @brief 带时间限制的求解
     *
     * @param time_limit 时间限制（秒）
     * @param warm_start 是否使用热启动
     * @return QP求解结果
     */
    QPSolution solveWithTimeLimit(double time_limit, bool warm_start = true);

    /**
     * @brief 更新求解器配置
     *
     * @param config 新的配置参数
     */
    void updateConfig(const QPSolverConfig& config);

    /**
     * @brief 获取当前配置
     *
     * @return 当前配置参数
     */
    const QPSolverConfig& getConfig() const { return config_; }

    /**
     * @brief 获取上一次的解
     *
     * @return 上一次的解向量
     */
    const Eigen::VectorXd& getLastSolution() const { return last_solution_; }

    /**
     * @brief 预处理问题数据
     *
     * 对矩阵进行缩放、重排序等预处理以提高求解效率
     *
     * @param H Hessian矩阵（将被修改）
     * @param A 约束矩阵（将被修改）
     * @param f 线性项向量（将被修改）
     * @param lower_bound 约束下界（将被修改）
     * @param upper_bound 约束上界（将被修改）
     */
    void preprocessProblem(Eigen::SparseMatrix<double>& H,
                          Eigen::SparseMatrix<double>& A,
                          Eigen::VectorXd& f,
                          Eigen::VectorXd& lower_bound,
                          Eigen::VectorXd& upper_bound);

    /**
     * @brief 验证解的可行性
     *
     * @param solution 待验证的解
     * @param tolerance 容差
     * @return 解是否可行
     */
    bool validateSolution(const Eigen::VectorXd& solution, double tolerance = 1e-6);

    /**
     * @brief 分析问题条件数
     *
     * @return 问题的条件数估计
     */
    double analyzeConditionNumber();

    /**
     * @brief 获取求解统计信息
     *
     * @return 求解统计信息向量
     */
    std::vector<double> getSolverStatistics();

    /**
     * @brief 重置求解器状态
     */
    void reset();

private:
    QPSolverConfig config_;                           ///< 求解器配置
    std::unique_ptr<OsqpEigen::Solver> solver_;       ///< OSQP求解器
    bool is_initialized_{false};                      ///< 是否已初始化
    Eigen::VectorXd last_solution_;                   ///< 上一次的解
    QPSolution last_result_;                          ///< 上一次的求解结果

    // 问题数据缓存（用于热启动）
    Eigen::SparseMatrix<double> H_cached_;            ///< 缓存的Hessian矩阵
    Eigen::VectorXd f_cached_;                        ///< 缓存的线性项
    Eigen::SparseMatrix<double> A_cached_;            ///< 缓存的约束矩阵
    Eigen::VectorXd lower_cached_;                    ///< 缓存的下界
    Eigen::VectorXd upper_cached_;                    ///< 缓存的上界

    /**
     * @brief 内部求解方法
     *
     * @param time_limit 时间限制，0表示无限制
     * @param warm_start 是否使用热启动
     * @return QP求解结果
     */
    QPSolution solveInternal(double time_limit, bool warm_start);

    /**
     * @brief 转换OSQP状态到字符串
     *
     * @param status OSQP状态码
     * @return 状态描述字符串
     */
    std::string statusToString(OsqpEigen::Status status);

    /**
     * @brief 检查问题数据的有效性
     *
     * @param H Hessian矩阵
     * @param f 线性项向量
     * @param A 约束矩阵
     * @param lower_bound 约束下界
     * @param upper_bound 约束上界
     * @return 数据是否有效
     */
    bool validateProblemData(const Eigen::SparseMatrix<double>& H,
                            const Eigen::VectorXd& f,
                            const Eigen::SparseMatrix<double>& A,
                            const Eigen::VectorXd& lower_bound,
                            const Eigen::VectorXd& upper_bound);

    /**
     * @brief 应用矩阵缩放
     *
     * @param matrix 要缩放的稀疏矩阵
     * @param row_scale 行缩放因子
     * @param col_scale 列缩放因子
     */
    void applyScaling(Eigen::SparseMatrix<double>& matrix,
                     const Eigen::VectorXd& row_scale,
                     const Eigen::VectorXd& col_scale);

    /**
     * @brief 估计矩阵范数
     *
     * @param matrix 稀疏矩阵
     * @param norm_type 范数类型（1, 2, 或 "inf"）
     * @return 矩阵范数
     */
    double estimateNorm(const Eigen::SparseMatrix<double>& matrix, const std::string& norm_type = "2");

    /**
     * @brief 检查矩阵的对称正定性
     *
     * @param matrix Hessian矩阵
     * @return 是否对称正定
     */
    bool isSymmetricPositiveDefinite(const Eigen::SparseMatrix<double>& matrix);

    /**
     * @brief 添加正则化项以确保正定性
     *
     * @param H Hessian矩阵（将被修改）
     * @param regularization 正则化系数
     */
    void addRegularization(Eigen::SparseMatrix<double>& H, double regularization = 1e-6);
};

/**
 * @brief QP问题构建器辅助类
 *
 * 提供便捷的接口来构建复杂的QP问题，支持分块构建、约束添加等功能。
 */
class QPProblemBuilder {
public:
    /**
     * @brief 构造函数
     *
     * @param n_vars 变量数量
     * @param n_constraints 约束数量
     */
    QPProblemBuilder(int n_vars, int n_constraints);

    /**
     * @brief 添加二次项
     *
     * @param i 变量索引i
     * @param j 变量索引j
     * @param value 系数值
     */
    void addQuadraticTerm(int i, int j, double value);

    /**
     * @brief 添加线性项
     *
     * @param i 变量索引
     * @param value 系数值
     */
    void addLinearTerm(int i, double value);

    /**
     * @brief 添加约束行
     *
     * @param constraint_index 约束索引
     * @param variable_index 变量索引
     * @param value 系数值
     */
    void addConstraintCoefficient(int constraint_index, int variable_index, double value);

    /**
     * @brief 设置约束边界
     *
     * @param constraint_index 约束索引
     * @param lower 下界
     * @param upper 上界
     */
    void setConstraintBounds(int constraint_index, double lower, double upper);

    /**
     * @brief 构建最终的QP问题数据
     *
     * @param H 输出的Hessian矩阵
     * @param f 输出的线性项向量
     * @param A 输出的约束矩阵
     * @param lower 输出的下界向量
     * @param upper 输出的上界向量
     */
    void build(Eigen::SparseMatrix<double>& H,
              Eigen::VectorXd& f,
              Eigen::SparseMatrix<double>& A,
              Eigen::VectorXd& lower,
              Eigen::VectorXd& upper);

    /**
     * @brief 获取当前变量数量
     */
    int getVariableCount() const { return n_vars_; }

    /**
     * @brief 获取当前约束数量
     */
    int getConstraintCount() const { return n_constraints_; }

private:
    int n_vars_;                                    ///< 变量数量
    int n_constraints_;                             ///< 约束数量

    // 三元组格式的矩阵数据
    std::vector<Eigen::Triplet<double>> h_triplets_; ///< Hessian矩阵三元组
    std::vector<Eigen::Triplet<double>> a_triplets_; ///< 约束矩阵三元组

    Eigen::VectorXd linear_terms_;                   ///< 线性项向量
    Eigen::VectorXd lower_bounds_;                   ///< 约束下界
    Eigen::VectorXd upper_bounds_;                   ///< 约束上界
};

} // namespace dpcbf_qp