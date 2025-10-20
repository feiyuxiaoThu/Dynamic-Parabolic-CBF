#pragma once
#include "bicycle_model.h"
#include "dpcbf.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <algorithm>
#include <cmath>

namespace dpcbf_qp {

/**
 * @brief Weights for the Quadratic Program (QP) objective function
 * 
 * The QP objective function minimizes control deviation from reference while
 * penalizing rapid control changes (jerk) for smooth behavior.
 * 
 * Objective: min 0.5 * (u - u_ref)^T * W * (u - u_ref) + jerk_penalty + rho * s^2
 */
struct QPWeights {
    double w_steer{12.0};          ///< Weight for steering angle deviation from reference
    double w_a{12.0};              ///< Weight for acceleration deviation from reference
    double w_jerk_steer{5.0};      ///< Weight for steering jerk penalty (smooth steering changes)
    double w_jerk_accel{200.0};    ///< Weight for acceleration jerk penalty (smooth acceleration changes)
    double rho{20.0};              ///< Weight for slack variable penalty (constraint violation cost)
};

/**
 * @brief Configuration parameters for discrete-time CBF constraints
 */
struct DiscreteCBFConfig {
    double gamma{0.25};    ///< Exponential decay rate for discrete CBF constraint (h_{k+1} - h_k + γ*h_k >= -s)
    double du{1e-3};       ///< Perturbation step size for numerical gradient computation
    double s_max{10.0};    ///< Upper bound for slack variable (maximum allowed constraint violation)
    
    // Options for obstacle consideration
    bool consider_all_obstacles{false};  ///< Whether to consider all obstacles instead of selection (true=use all obstacles, false=use selection logic)
    double h_threshold{0.2};             ///< Threshold below which obstacles are always included (when not using all obstacles mode)
    int max_obstacles{3};                ///< Maximum number of obstacles to consider if not using all obstacles mode
};

/**
 * @brief QP-based controller using Dynamic Parabolic Control Barrier Functions
 * 
 * Implements a Control Barrier Function based Quadratic Program (CBF-QP) that
 * ensures safety by constraining the control inputs such that the DPCBF value
 * remains non-negative, which guarantees forward invariance of the safe set.
 * 
 * The controller uses a single slack variable approach for constraint relaxation,
 * allowing feasible solutions even in challenging scenarios while maintaining
 * safety as the primary objective.
 * 
 * QP Formulation:
 * min: (steering - steering_ref)² * w_steer + (accel - accel_ref)² * w_accel 
 *      + steering_jerk² * w_jerk_steer + accel_jerk² * w_jerk_accel
 *      + slack² * rho
 * s.t.: DPCBF constraint for obstacles: ∇h*u ≥ -h - γ*h - s
 *       Control limits: |steering| ≤ max_steering, |accel| ≤ max_accel  
 *       Slack variable limits: 0 ≤ s ≤ s_max
 */
class QPController {
public:
    /**
     * @brief Constructor for QP controller
     * 
     * @param model Reference to the bicycle model for state prediction
     * @param dparams DPCBF parameters for safety constraint computation
     * @param w QP objective weights
     * @param cfg Discrete CBF configuration parameters
     */
    QPController(const BicycleModel& model,
                 const DPCBFParams& dparams,
                 const QPWeights& w,
                 const DiscreteCBFConfig& cfg);

    /**
     * @brief Solve the CBF-QP optimization problem to compute safe control input
     * 
     * The controller:
     * 1. Selects most critical obstacles based on current DPCBF values
     * 2. Constructs linearized DPCBF constraints for these obstacles
     * 3. Solves the QP to find control input that minimizes deviation from reference
     *    while satisfying safety constraints
     * 
     * @param s Current robot state [x, y, θ, v]
     * @param u_ref Reference control input (nominal controller output)
     * @param obstacles Vector of obstacles to consider for safety
     * @return Safe control input [steering, acceleration] that satisfies CBF constraints
     */
    Control solve(const State& s, const Control& u_ref,
                  const std::vector<Obstacle>& obstacles) const;
    
    /**
     * @brief Update the previous control input for jerk computation
     * 
     * Used to maintain control continuity and smooth transitions by penalizing
     * rapid changes in control inputs (jerk).
     * 
     * @param u_prev Previous control input [steering, acceleration]
     */
    void updatePreviousControl(const Control& u_prev);

private:
    const BicycleModel& model_;      ///< Reference to bicycle model for state prediction
    DPCBFParams dparams_;            ///< DPCBF parameters (k_lambda, k_mu, margin)
    QPWeights w_;                    ///< QP objective weights
    DiscreteCBFConfig cfg_;          ///< Discrete CBF configuration
    
    // Storage for previous control inputs to compute jerk penalties
    mutable Control u_previous_{0.0, 0.0};  ///< Previous control input [steering, acceleration]
    mutable bool has_previous_{false};       ///< Flag indicating if previous control is available
};

} // namespace dpcbf_qp