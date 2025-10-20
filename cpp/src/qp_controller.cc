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

    // Number of decision variables: [steering, acceleration, slack]
    const int nv = 3;

    // Determine how many obstacles to process based on configuration
    int m_obs;
    std::vector<int> obstacle_indices;
    
    if (cfg_.consider_all_obstacles) {
        // Process ALL obstacles
        m_obs = (int)obstacles.size();
        obstacle_indices.resize(m_obs);
        for (int i = 0; i < m_obs; ++i) {
            obstacle_indices[i] = i;
        }
    } else {
        // Use the original selection logic with configurable parameters
        // Threshold for obstacle selection: obstacles with h < h_thresh are considered critical
        const double h_thresh = cfg_.h_threshold;
        // Maximum number of obstacles to include in constraints (to limit QP size)
        const int K = cfg_.max_obstacles;

        // Select critical obstacles based on DPCBF values
        std::vector<int> sel;  // Selected obstacle indices
        sel.reserve(obstacles.size());
        
        // Score each obstacle by its DPCBF value (lower h means more dangerous)
        std::vector<std::pair<double,int>> scores;  // (h_value, obstacle_index)
        scores.reserve(obstacles.size());
        
        // Compute DPCBF value for each obstacle to assess criticality
        for (int i = 0; i < (int)obstacles.size(); ++i) {
            auto hres = dpcbf_continuous(s.x, s.y, s.theta, s.v, obstacles[i], model_.spec().radius, dparams_);
            scores.emplace_back(hres.h, i);
        }
        
        // Sort obstacles by safety (ascending order: most dangerous first)
        std::sort(scores.begin(), scores.end(),
                  [](const auto& a, const auto& b){ return a.first < b.first; });

        // First, select all obstacles that are in critical danger (h < h_thresh)
        for (const auto& pr : scores) {
            if (pr.first < h_thresh) sel.push_back(pr.second);
        }
        
        // Then, if we haven't reached the limit, add the most critical obstacles up to K
        int cap = std::min(K, (int)obstacles.size());
        for (int i = 0; i < cap && (int)sel.size() < cap; ++i) {
            int idx = scores[i].second;
            // Only add if not already selected
            if (std::find(sel.begin(), sel.end(), idx) == sel.end()) {
                sel.push_back(idx);
            }
        }
        
        m_obs = (int)sel.size();
        obstacle_indices = sel;
    }
    
    const int m_total = m_obs + 3;  // +3 for: steering limits, acceleration limits, slack limits

    // Initialize QP matrices for the canonical form: min 0.5 * x^T * H * x + f^T * x
    MatrixXd H = MatrixXd::Zero(nv, nv);  // Hessian matrix (quadratic terms)
    VectorXd f = VectorXd::Zero(nv);      // Linear terms in objective
    
    // Set up the quadratic cost terms for control tracking
    H(0,0) = w_.w_steer;  // Weight for steering angle deviation from reference
    H(1,1) = w_.w_a;      // Weight for acceleration deviation from reference
    H(2,2) = w_.rho;      // Weight for slack variable (constraint violation penalty)
    
    // Set up linear terms for reference tracking: -H * x_ref
    VectorXd xref(nv);
    xref << u_ref.steer, u_ref.a, 0.0;  // Reference control with zero slack
    f = -H * xref;
    
    // Add jerk penalty terms to ensure smooth control transitions
    // Minimize (u_current - u_previous)² for both steering and acceleration
    if (has_previous_) {
        // Steering jerk penalty: w_jerk_steer * (steer - steer_prev)²
        H(0,0) += w_.w_jerk_steer;
        f(0) += -w_.w_jerk_steer * u_previous_.steer;
        
        // Acceleration jerk penalty: w_jerk_accel * (accel - accel_prev)²  
        H(1,1) += w_.w_jerk_accel;
        f(1) += -w_.w_jerk_accel * u_previous_.a;
    }

    // Constraint matrix A and bounds vectors l, u for canonical form: l <= A*x <= u
    MatrixXd A = MatrixXd::Zero(m_total, nv);
    VectorXd l = VectorXd::Constant(m_total, -OsqpEigen::INFTY);  // Lower bounds
    VectorXd ub = VectorXd::Constant(m_total,  OsqpEigen::INFTY); // Upper bounds

    // Build DPCBF constraints for selected obstacles
    for (int i = 0; i < m_obs; ++i) {
        const auto& obs = obstacles[obstacle_indices[i]];  // Get the selected obstacle
        
        // Predict next state using reference control (for linearization point)
        State s1 = model_.step(s, u_ref);
        
        // Compute discrete DPCBF values: [h_current, h_next - h_current]
        auto hd0 = dpcbf_discrete(s.x, s.y, s.theta, s.v, s1.x, s1.y, s1.theta, s1.v, obs, model_.spec().radius, dparams_);
        // Compute linearization constant: c0 = (h_{k+1} - h_k) + γ*h_k
        double c0 = hd0[1] + cfg_.gamma * hd0[0];

        // Compute gradient of DPCBF constraint w.r.t. steering using finite differences
        Control u_steer = u_ref; u_steer.steer += cfg_.du;  // Perturb steering
        State s_steer = model_.step(s, u_steer);
        auto hd_steer = dpcbf_discrete(s.x, s.y, s.theta, s.v, s_steer.x, s_steer.y, s_steer.theta, s_steer.v, obs, model_.spec().radius, dparams_);
        double c_steer = hd_steer[1] + cfg_.gamma * hd_steer[0];
        double J_steer = (c_steer - c0) / cfg_.du;  // ∂(constraint)/∂(steering)

        // Compute gradient of DPCBF constraint w.r.t. acceleration using finite differences
        Control u_a = u_ref; u_a.a += cfg_.du;  // Perturb acceleration
        State s_a = model_.step(s, u_a);
        auto hd_a = dpcbf_discrete(s.x, s.y, s.theta, s.v, s_a.x, s_a.y, s_a.theta, s_a.v, obs, model_.spec().radius, dparams_);
        double c_a = hd_a[1] + cfg_.gamma * hd_a[0];
        double J_a = (c_a - c0) / cfg_.du;  // ∂(constraint)/∂(acceleration)

        // Set up the constraint: J_steer * steer + J_a * accel + 1.0 * slack >= -c0
        // In canonical form: -J_steer * steer - J_a * accel - 1.0 * slack <= c0
        A(i,0) = J_steer;  // Coefficient for steering
        A(i,1) = J_a;      // Coefficient for acceleration
        A(i,2) = 1.0;      // Coefficient for slack variable
        // For constraint: J*δu >= -c0 (where δu = u - u_ref)
        // This becomes: J*steer + J*accel + slack >= -c0 + J*uref
        // Or: J*steer + J*accel + slack >= lower_bound
        l(i) = -c0 + J_steer*u_ref.steer + J_a*u_ref.a;  // Lower bound for safe set
    }

    // Add steering angle limits: steer_lo <= steering <= steer_hi
    double steer_lo = -model_.spec().steer_max;
    double steer_hi =  model_.spec().steer_max;
    A(m_obs+0, 0) = 1.0; l(m_obs+0) = steer_lo; ub(m_obs+0) = steer_hi;

    // Add acceleration limits: a_lo <= acceleration <= a_hi
    double a_lo = -model_.spec().a_max;
    double a_hi =  model_.spec().a_max;
    A(m_obs+1, 1) = 1.0; l(m_obs+1) = a_lo; ub(m_obs+1) = a_hi;

    // Add slack variable limits: 0 <= slack <= cfg_.s_max
    A(m_obs+2, 2) = 1.0; l(m_obs+2) = 0.0; ub(m_obs+2) = cfg_.s_max;

    // Setup and solve the QP using OSQP
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);      // Use warm start for faster convergence
    solver.settings()->setVerbosity(false);     // Disable solver output
    solver.data()->setNumberOfVariables(nv);    // Set number of decision variables
    solver.data()->setNumberOfConstraints(m_total); // Set number of constraints

    // Convert dense matrices to sparse format for OSQP
    Eigen::SparseMatrix<double> Hs = H.sparseView();
    Eigen::SparseMatrix<double> As = A.sparseView();

    // Load the QP problem into the solver
    (void)solver.data()->setHessianMatrix(Hs);
    (void)solver.data()->setGradient(f);
    (void)solver.data()->setLinearConstraintsMatrix(As);
    (void)solver.data()->setLowerBound(l);
    (void)solver.data()->setUpperBound(ub);

    // Initialize and solve the QP
    if (!solver.initSolver()) {
        // If initialization fails, return reference control
        return u_ref;
    }
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        // If solving fails, return reference control
        return u_ref;
    }

    // Extract the optimal solution
    Eigen::VectorXd sol = solver.getSolution();

    // Create and clamp the final control output
    Control u;
    u.steer = BicycleModel::clamp(sol(0), steer_lo, steer_hi);  // Clamp steering
    u.a     = BicycleModel::clamp(sol(1), a_lo, a_hi);          // Clamp acceleration
    
    // Update control history for next iteration's jerk penalty
    u_previous_ = u;
    has_previous_ = true;
    
    return u;
}

void QPController::updatePreviousControl(const Control& u_prev) {
    u_previous_ = u_prev;
    has_previous_ = true;
}

} // namespace dpcbf_qp