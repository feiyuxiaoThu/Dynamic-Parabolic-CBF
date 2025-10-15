#include "../include/qp_controller.h"
#include <algorithm>
#include <vector>

namespace dpcbf_qp {

QPController::QPController(const BicycleModel& model,
                           const DPCBFParams& dparams,
                           const QPWeights& w,
                           const DiscreteCBFConfig& cfg)
    : model_(model), dparams_(dparams), w_(w), cfg_(cfg) {}

Control QPController::solve(const State& s, const Control& u_ref,
                  const std::vector<Obstacle>& obstacles) const {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const int nv = 3;

    // 简化权重：只保留控制输入跟踪权重
    const double w_steer = 1.0;  // 转向角跟踪权重
    const double w_accel = 1.0;  // 加速度跟踪权重
    const double h_thresh = 0.2;
    const int K = 3;

    std::vector<int> sel;
    sel.reserve(obstacles.size());
    std::vector<std::pair<double,int>> scores;
    scores.reserve(obstacles.size());
    for (int i = 0; i < (int)obstacles.size(); ++i) {
        auto hres = dpcbf_continuous(s.x, s.y, s.theta, s.v, obstacles[i], model_.spec().radius, dparams_);
        scores.emplace_back(hres.h, i);
    }
    std::sort(scores.begin(), scores.end(),
              [](const auto& a, const auto& b){ return a.first < b.first; });

    for (const auto& pr : scores) {
        if (pr.first < h_thresh) sel.push_back(pr.second);
    }
    int cap = std::min(K, (int)obstacles.size());
    for (int i = 0; i < cap && (int)sel.size() < cap; ++i) {
        int idx = scores[i].second;
        if (std::find(sel.begin(), sel.end(), idx) == sel.end()) {
            sel.push_back(idx);
        }
    }

    const int m_obs = (int)sel.size();
    const int m_total = m_obs + 3;

    MatrixXd H = MatrixXd::Zero(nv, nv);
    VectorXd f = VectorXd::Zero(nv);
    
    // 基本控制跟踪权重
    H(0,0) = w_.w_steer;
    H(1,1) = w_.w_a;
    H(2,2) = w_.rho;
    
    // 基本参考跟踪项
    VectorXd xref(nv);
    xref << u_ref.steer, u_ref.a, 0.0;
    f = -H * xref;
    
    // 添加jerk惩罚项（如果有上一时刻的控制输入）
    if (has_previous_) {
        // jerk = (u_current - u_previous) / dt
        // 惩罚项: w_jerk * jerk^2 = w_jerk * (u_current - u_previous)^2 / dt^2
        // 这里假设dt=1（或者将dt^2合并到权重中）
        
        // 转向角jerk惩罚: w_jerk_steer * (steer - steer_prev)^2
        H(0,0) += w_.w_jerk_steer;
        f(0) += -w_.w_jerk_steer * u_previous_.steer;
        
        // 加速度jerk惩罚: w_jerk_accel * (accel - accel_prev)^2  
        H(1,1) += w_.w_jerk_accel;
        f(1) += -w_.w_jerk_accel * u_previous_.a;
    }

    // 移除复杂的轨迹跟踪计算
    // 现在只使用基于回归轨迹计算的 u_ref 作为控制参考
    // 这样更符合实际情况：车辆因避障偏离轨迹后，应该跟踪回归轨迹的控制量

    MatrixXd A = MatrixXd::Zero(m_total, nv);
    VectorXd l = VectorXd::Constant(m_total, -OsqpEigen::INFTY);
    VectorXd ub = VectorXd::Constant(m_total,  OsqpEigen::INFTY);

    for (int i=0; i<m_obs; ++i) {
        const auto& obs = obstacles[ sel[i] ];
        State s1 = model_.step(s, u_ref);
        auto hd0 = dpcbf_discrete(s.x, s.y, s.theta, s.v, s1.x, s1.y, s1.theta, s1.v, obs, model_.spec().radius, dparams_);
        double c0 = hd0[1] + cfg_.gamma * hd0[0];

        Control u_steer = u_ref; u_steer.steer += cfg_.du;
        State s_steer = model_.step(s, u_steer);
        auto hd_steer = dpcbf_discrete(s.x, s.y, s.theta, s.v, s_steer.x, s_steer.y, s_steer.theta, s_steer.v, obs, model_.spec().radius, dparams_);
        double c_steer = hd_steer[1] + cfg_.gamma * hd_steer[0];

        Control u_a = u_ref; u_a.a += cfg_.du;
        State s_a = model_.step(s, u_a);
        auto hd_a = dpcbf_discrete(s.x, s.y, s.theta, s.v, s_a.x, s_a.y, s_a.theta, s_a.v, obs, model_.spec().radius, dparams_);
        double c_a = hd_a[1] + cfg_.gamma * hd_a[0];

        double J_steer = (c_steer - c0) / cfg_.du;
        double J_a     = (c_a     - c0) / cfg_.du;

        A(i,0) = J_steer;
        A(i,1) = J_a;
        A(i,2) = 1.0;
        l(i)   = -c0 + J_steer*u_ref.steer + J_a*u_ref.a;
    }

    double steer_lo = -model_.spec().steer_max;
    double steer_hi =  model_.spec().steer_max;
    A(m_obs+0, 0) = 1.0; l(m_obs+0) = steer_lo; ub(m_obs+0) = steer_hi;

    double a_lo = -model_.spec().a_max;
    double a_hi =  model_.spec().a_max;
    A(m_obs+1, 1) = 1.0; l(m_obs+1) = a_lo; ub(m_obs+1) = a_hi;

    A(m_obs+2, 2) = 1.0; l(m_obs+2) = 0.0; ub(m_obs+2) = cfg_.s_max;

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(nv);
    solver.data()->setNumberOfConstraints(m_total);

    Eigen::SparseMatrix<double> Hs = H.sparseView();
    Eigen::SparseMatrix<double> As = A.sparseView();

    (void)solver.data()->setHessianMatrix(Hs);
    (void)solver.data()->setGradient(f);
    (void)solver.data()->setLinearConstraintsMatrix(As);
    (void)solver.data()->setLowerBound(l);
    (void)solver.data()->setUpperBound(ub);

    if (!solver.initSolver()) {
        return u_ref;
    }
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return u_ref;
    }

    Eigen::VectorXd sol = solver.getSolution();

    Control u;
    u.steer = BicycleModel::clamp(sol(0), steer_lo, steer_hi);
    u.a     = BicycleModel::clamp(sol(1), a_lo, a_hi);
    
    // 更新控制历史用于下一次jerk计算
    u_previous_ = u;
    has_previous_ = true;
    
    return u;
}

void QPController::updatePreviousControl(const Control& u_prev) {
    u_previous_ = u_prev;
    has_previous_ = true;
}

} // namespace dpcbf_qp