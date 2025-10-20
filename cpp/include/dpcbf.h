#pragma once
#include <cmath>
#include <array>

namespace dpcbf_qp {

/**
 * @brief Structure representing a dynamic obstacle with position, radius, and velocity
 */
struct Obstacle {
    double ox{0.0}, oy{0.0};  ///< Obstacle position (x, y coordinates)
    double r{0.0};            ///< Obstacle radius
    double vx{0.0}, vy{0.0};  ///< Obstacle velocity (x, y components)
};

/**
 * @brief Parameters for Dynamic Parabolic Control Barrier Function
 * 
 * The DPCBF parameters control the shape and behavior of the parabolic safety boundary.
 * The parabolic boundary adapts based on distance and relative velocity to obstacles.
 */
struct DPCBFParams {
    double k_lambda{0.1};     ///< Scaling factor for lateral velocity penalty (v_y^2 term)
    double k_mu{0.5};         ///< Scaling factor for distance-based safety margin (constant term)
    double margin{1.05};      ///< Safety margin coefficient (safety radius multiplier)
    double eps{1e-6};         ///< Small value for numerical stability to avoid division by zero
};

/**
 * @brief Result structure for DPCBF computation
 * 
 * Contains the computed DPCBF value 'h' which defines the safety boundary.
 * h >= 0 indicates safe state, h < 0 indicates unsafe state.
 */
struct DPCBFResult {
    double h{0.0};            ///< DPCBF value: h >= 0 for safety, h < 0 for unsafe
};

/**
 * @brief Compute continuous-time Dynamic Parabolic Control Barrier Function value
 * 
 * Implements the Dynamic Parabolic CBF (DPCBF) in continuous time. The safety boundary
 * is defined by a parabola in the Line-of-Sight (LOS) coordinate system that adapts
 * based on distance and relative velocity to obstacles.
 * 
 * Mathematical formulation:
 * h(x) = v_x + λ(x) * v_y^2 + μ(x)
 * where:
 * - v_x: relative velocity in the LOS direction (radial component)
 * - v_y: relative velocity perpendicular to LOS direction (tangential component)  
 * - λ(x): dynamic curvature parameter for the parabolic boundary
 * - μ(x): dynamic offset parameter for the safety margin
 * 
 * The parameters λ and μ adapt based on the safety distance and relative velocity magnitude:
 * - As distance increases, the parabolic boundary expands to allow more maneuvering space
 * - As relative velocity increases, the parabola becomes more open to allow lateral movement
 * 
 * @param X Current robot x position
 * @param Y Current robot y position
 * @param theta Current robot orientation
 * @param v Current robot velocity magnitude
 * @param obs The obstacle to check against
 * @param robot_radius Robot's collision radius
 * @param p DPCBF parameters
 * @return DPCBFResult containing the computed h value
 */
DPCBFResult dpcbf_continuous(double X, double Y, double theta, double v,
                                    const Obstacle& obs, double robot_radius,
                                    const DPCBFParams& p);

/**
 * @brief Compute discrete-time DPCBF values for QP constraint formulation
 * 
 * Computes the DPCBF values at the current and next time steps to form the
 * discrete-time constraint: h_{k+1} - h_k + γ*h_k >= 0, which ensures
 * forward invariance of the safe set in discrete time.
 * 
 * @param X, Y, theta, v Current robot state (position, orientation, velocity)
 * @param X1, Y1, theta1, v1 Predicted robot state at next time step
 * @param obs The obstacle to check against
 * @param robot_radius Robot's collision radius
 * @param p DPCBF parameters
 * @return std::array<double,2> containing {h_k, h_{k+1} - h_k} for constraint formation
 */
std::array<double,2> dpcbf_discrete(double X, double Y, double theta, double v,
                                            double X1, double Y1, double theta1, double v1,
                                            const Obstacle& obs, double robot_radius,
                                            const DPCBFParams& p);

} // namespace dpcbf_qp