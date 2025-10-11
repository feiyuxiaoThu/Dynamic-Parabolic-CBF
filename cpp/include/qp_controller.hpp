#pragma once
#include "bicycle_model.hpp"
#include "dpcbf.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <algorithm>
#include <cmath>

// 纯 CBF-QP（单松弛 s）：
// min 0.5 (x - x_ref)^T W (x - x_ref)
// s.t. 变量盒约束 + 线性化的离散 DPCBF 约束
// x = [steer, a, s]，s为松弛变量(>=0)
struct QPWeights {
    double w_steer{12.0};
    double w_a{12.0};
    double rho{20.0}; // 松弛变量权重
};

struct DiscreteCBFConfig {
    double gamma{0.25};
    double du{1e-3};      // 数值梯度步长
    double s_max{10.0};   // 松弛变量上界
};

class QPController {
public:
    QPController(const BicycleModel& model,
                 const DPCBFParams& dparams,
                 const QPWeights& w,
                 const DiscreteCBFConfig& cfg)
    : model_(model), dparams_(dparams), w_(w), cfg_(cfg) {}

    Control solve(const State& s, const Control& u_ref,
                  const std::vector<Obstacle>& obstacles,
                  double ref_x, double ref_y, double theta_ref) const {
        using Eigen::MatrixXd;
        using Eigen::VectorXd;

        const int nv = 3;

        // 参考跟踪权重（增强位置/航向贴合）
        const double w_px = 3.0, w_py = 3.0, w_th = 1.5;

        // 自适应障碍选择：h<thresh 的全纳入 + 兜底前K个
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

        // 构造目标：0.5 x^T W x + f^T x
        MatrixXd H = MatrixXd::Zero(nv, nv);
        VectorXd f = VectorXd::Zero(nv);
        H(0,0) = w_.w_steer;
        H(1,1) = w_.w_a;
        H(2,2) = w_.rho;
        VectorXd xref(nv);
        xref << u_ref.steer, u_ref.a, 0.0;
        f = -H * xref;

        // 参考轨迹一步线性化跟踪代价
        // 基线下一步
        State s1_ref = model_.step(s, u_ref);
        // 数值差分雅可比
        Control u_steer_ref = u_ref; u_steer_ref.steer += cfg_.du;
        State s1_steer = model_.step(s, u_steer_ref);
        Control u_a_ref = u_ref; u_a_ref.a += cfg_.du;
        State s1_a = model_.step(s, u_a_ref);

        Eigen::Vector2d Jx, Jy, Jth;
        Jx  << (s1_steer.x  - s1_ref.x)/cfg_.du,  (s1_a.x  - s1_ref.x)/cfg_.du;
        Jy  << (s1_steer.y  - s1_ref.y)/cfg_.du,  (s1_a.y  - s1_ref.y)/cfg_.du;
        Jth << (s1_steer.theta - s1_ref.theta)/cfg_.du, (s1_a.theta - s1_ref.theta)/cfg_.du;

        double e_th = BicycleModel::wrapAngle(s1_ref.theta - theta_ref);
        Eigen::Vector3d e0;
        e0 << (s1_ref.x - ref_x), (s1_ref.y - ref_y), e_th;

        // E: 3x2
        Eigen::Matrix<double,3,2> E;
        E.row(0) = Jx.transpose();
        E.row(1) = Jy.transpose();
        E.row(2) = Jth.transpose();

        Eigen::Matrix3d Wtrk = Eigen::Matrix3d::Zero();
        Wtrk(0,0)=w_px; Wtrk(1,1)=w_py; Wtrk(2,2)=w_th;

        // H_add（对 steer,a），扩展到 3 维（第三维 s 为 0）
        Eigen::Matrix2d Haa = E.transpose() * Wtrk * E;
        H(0,0) += Haa(0,0);
        H(0,1) += Haa(0,1);
        H(1,0) += Haa(1,0);
        H(1,1) += Haa(1,1);
        // f_add
        Eigen::Vector2d fa2 = E.transpose() * Wtrk * e0;
        f(0) += fa2(0) - (Haa.row(0) * Eigen::Vector2d(u_ref.steer, u_ref.a))(0);
        f(1) += fa2(1) - (Haa.row(1) * Eigen::Vector2d(u_ref.steer, u_ref.a))(0);

        // 约束矩阵 A, 下界 l, 上界 ub
        MatrixXd A = MatrixXd::Zero(m_total, nv);
        VectorXd l = VectorXd::Constant(m_total, -OsqpEigen::INFTY);
        VectorXd ub = VectorXd::Constant(m_total,  OsqpEigen::INFTY);

        // 线性化 CBF 离散约束：c_i ≈ c0 + J*(u - u0) + s >= 0
        // 选择 u0 = u_ref 单次线性化；c = (h1 - h0) + gamma*h0
        for (int i=0; i<m_obs; ++i) {
            const auto& obs = obstacles[ sel[i] ];

            // baseline at u0
            State s1 = model_.step(s, u_ref);
            auto hd0 = dpcbf_discrete(s.x, s.y, s.theta, s.v,
                                      s1.x, s1.y, s1.theta, s1.v,
                                      obs, model_.spec().radius, dparams_);
            double c0 = hd0[1] + cfg_.gamma * hd0[0];

            // numerical gradient w.r.t steer
            Control u_steer = u_ref; u_steer.steer += cfg_.du;
            State s_steer = model_.step(s, u_steer);
            auto hd_steer = dpcbf_discrete(s.x, s.y, s.theta, s.v,
                                           s_steer.x, s_steer.y, s_steer.theta, s_steer.v,
                                           obs, model_.spec().radius, dparams_);
            double c_steer = hd_steer[1] + cfg_.gamma * hd_steer[0];

            // numerical gradient w.r.t a
            Control u_a = u_ref; u_a.a += cfg_.du;
            State s_a = model_.step(s, u_a);
            auto hd_a = dpcbf_discrete(s.x, s.y, s.theta, s.v,
                                       s_a.x, s_a.y, s_a.theta, s_a.v,
                                       obs, model_.spec().radius, dparams_);
            double c_a = hd_a[1] + cfg_.gamma * hd_a[0];

            double J_steer = (c_steer - c0) / cfg_.du;
            double J_a     = (c_a     - c0) / cfg_.du;

            // A_i * x in [l_i, +inf], A_i = [J_steer, J_a, 1], l_i = -c0 + J*u_ref
            A(i,0) = J_steer;
            A(i,1) = J_a;
            A(i,2) = 1.0;
            l(i)   = -c0 + J_steer*u_ref.steer + J_a*u_ref.a;
        }

        // 变量盒约束：steer ∈ [-steer_max, steer_max]
        double steer_lo = -model_.spec().steer_max;
        double steer_hi =  model_.spec().steer_max;
        A(m_obs+0, 0) = 1.0; A(m_obs+0, 1) = 0.0; A(m_obs+0, 2) = 0.0;
        l(m_obs+0) = steer_lo; ub(m_obs+0) = steer_hi;

        // a ∈ [-a_max, a_max]
        double a_lo = -model_.spec().a_max;
        double a_hi =  model_.spec().a_max;
        A(m_obs+1, 0) = 0.0; A(m_obs+1, 1) = 1.0; A(m_obs+1, 2) = 0.0;
        l(m_obs+1) = a_lo;    ub(m_obs+1) = a_hi;

        // s ∈ [0, s_max]
        A(m_obs+2, 0) = 0.0; A(m_obs+2, 1) = 0.0; A(m_obs+2, 2) = 1.0;
        l(m_obs+2) = 0.0;    ub(m_obs+2) = cfg_.s_max;

        // 设置 OSQP-Eigen
        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(nv);
        solver.data()->setNumberOfConstraints(m_total);

        // 转为稀疏矩阵
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
        return u;
    }

private:
    const BicycleModel& model_;
    DPCBFParams dparams_;
    QPWeights w_;
    DiscreteCBFConfig cfg_;
};