#include "../include/dpcbf.h"
#include <algorithm> // for std::max
#include <cmath>     // for sqrt, atan2, cos, sin

namespace {
    /**
     * @brief Compute 2D Euclidean norm
     * 
     * @param x x-component of the vector
     * @param y y-component of the vector
     * @return The Euclidean norm of the vector (sqrt(x^2 + y^2))
     */
    double norm2(double x, double y) {
        return std::sqrt(x*x + y*y);
    }

    /**
     * @brief Rotate velocity vector to Line-of-Sight (LOS) coordinate system
     * 
     * Transforms the relative velocity vector from global coordinates to the
     * LOS coordinate system where:
     * - x-axis points from robot to obstacle (radial direction)
     * - y-axis is perpendicular to x-axis (tangential direction)
     * 
     * This transformation enables the parabolic safety boundary to be defined
     * in a coordinate system that naturally separates radial and tangential
     * components of relative motion.
     * 
     * @param px x-component of relative position vector (obstacle relative to robot)
     * @param py y-component of relative position vector
     * @param vx x-component of relative velocity vector
     * @param vy y-component of relative velocity vector
     * @param[out] vxl x-component of velocity in LOS coordinates (radial velocity)
     * @param[out] vyl y-component of velocity in LOS coordinates (tangential velocity)
     */
    void rotateToLOS(double px, double py, double vx, double vy, double& vxl, double& vyl) {
        double rot = std::atan2(py, px);  // Angle from robot to obstacle
        double c = std::cos(rot), s = std::sin(rot);
        
        // Apply rotation matrix to transform velocity to LOS frame
        vxl = c*vx + s*vy;  // Radial component (along the line from robot to obstacle)
        vyl = -s*vx + c*vy; // Tangential component (perpendicular to radial direction)
    }
} // namespace

namespace dpcbf_qp {

DPCBFResult dpcbf_continuous(double X, double Y, double theta, double v,
                                    const Obstacle& obs, double robot_radius,
                                    const DPCBFParams& p) {
    // Compute relative position vector from robot to obstacle
    double px = obs.ox - X;  // x-component of relative position
    double py = obs.oy - Y;  // y-component of relative position
    
    // Compute relative velocity vector (obstacle velocity minus robot velocity)
    double vx_rel = obs.vx - v*std::cos(theta);  // x-component of relative velocity
    double vy_rel = obs.vy - v*std::sin(theta);  // y-component of relative velocity

    // Compute magnitudes of relative position and velocity
    double pr = norm2(px, py);  // Distance from robot to obstacle
    double vr = norm2(vx_rel, vy_rel);  // Magnitude of relative velocity

    // Compute effective collision radius with safety margin
    // ego_dim represents the minimum safe distance between robot and obstacle
    double ego_dim = (obs.r + robot_radius) * p.margin;
    
    // Compute safety distance: d_safe = max(||p_rel||^2 - ego_dim^2, ε)
    // This represents how far the current position is from the minimum safety boundary
    double dsafe = std::max(pr*pr - ego_dim*ego_dim, p.eps);

    // Transform relative velocity to Line-of-Sight (LOS) coordinate system
    // This separates radial (vxl) and tangential (vyl) components of relative motion
    double vxl=0.0, vyl=0.0;
    rotateToLOS(px, py, vx_rel, vy_rel, vxl, vyl);

    // Compute scale factor that adjusts the dynamic parameters based on geometry
    // The scale factor normalizes the parameters based on the obstacle dimensions
    double scale = std::sqrt(p.margin*p.margin - 1.0) / (ego_dim + p.eps);
    
    // Compute dynamic parameters λ (lambda) and μ (mu) that define the parabolic boundary
    // These parameters adapt based on safety distance and relative velocity:
    // - As distance increases (dsafe increases), both parameters increase (larger safety region)
    // - As relative velocity increases (vr increases), λ decreases (allows more lateral movement)
    double lam = (p.k_lambda * scale) * std::sqrt(dsafe) / (vr + p.eps);  // Adaptive curvature parameter
    double mu  = (p.k_mu     * scale) * std::sqrt(dsafe);                 // Adaptive offset parameter

    // Compute DPCBF value: h(x) = v_x + λ * v_y^2 + μ
    // where:
    // - v_x (vxl): radial velocity component (positive if moving away from obstacle)
    // - λ * v_y^2: penalty for tangential velocity (quadratic in tangential velocity)
    // - μ: distance-based safety margin (increases with distance to obstacle)
    DPCBFResult res;
    res.h = vxl + lam * (vyl*vyl) + mu;
    
    return res;
}

std::array<double,2> dpcbf_discrete(double X, double Y, double theta, double v,
                                            double X1, double Y1, double theta1, double v1,
                                            const Obstacle& obs, double robot_radius,
                                            const DPCBFParams& p) {
    // Compute DPCBF value at current time step k
    auto hk = dpcbf_continuous(X, Y, theta, v, obs, robot_radius, p).h;
    
    // Compute DPCBF value at next time step k+1
    auto h1 = dpcbf_continuous(X1, Y1, theta1, v1, obs, robot_radius, p).h;
    
    // Return current value and discrete difference for constraint formulation
    // The discrete constraint uses: h_{k+1} - h_k + γ*h_k >= 0
    return {hk, h1 - hk};
}

} // namespace dpcbf_qp