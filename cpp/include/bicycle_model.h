#pragma once
#include <cmath>

namespace dpcbf_qp {

/**
 * @brief Robot state representation [x, y, theta, v]
 * 
 * Contains the essential state variables for the kinematic bicycle model:
 * - Position (x, y) in global coordinates
 * - Orientation theta (heading angle) in radians
 * - Velocity v (forward speed) in m/s
 */
struct State {
    double x{0.0}, y{0.0};     ///< Position coordinates in global frame
    double theta{0.0};         ///< Heading angle in radians
    double v{0.0};             ///< Forward velocity in m/s
};

/**
 * @brief Control input representation [steering, acceleration]
 * 
 * Contains the control inputs for the bicycle model:
 * - Steering angle of the front wheel
 * - Longitudinal acceleration
 */
struct Control {
    double steer{0.0};         ///< Steering angle of front wheel in radians
    double a{0.0};             ///< Longitudinal acceleration in m/s²
};

/**
 * @brief Robot specification parameters
 * 
 * Contains physical and kinematic constraints of the robot platform
 */
struct RobotSpec {
    double radius{0.3};        ///< Collision radius of the robot in meters
    double a_max{5.0};         ///< Maximum longitudinal acceleration/deceleration in m/s²
    double steer_max{0.5};     ///< Maximum absolute steering angle in radians
    double L{1.0};             ///< Wheelbase length (distance between front and rear axles) in meters
};

/**
 * @brief Kinematic Bicycle Model Implementation
 * 
 * Implements the kinematic bicycle model which approximates the motion of a car-like
 * vehicle. This model treats the vehicle as a rigid body with a front wheel that can
 * steer and a rear wheel that follows. It's commonly used in robotics and autonomous
 * driving for path planning and control due to its simplicity and reasonable accuracy
 * at moderate speeds.
 * 
 * State equations:
 * - ẋ = v * cos(θ)
 * - ẏ = v * sin(θ) 
 * - θ̇ = (v / L) * tan(δ)  (where δ is steering angle)
 * - v̇ = a
 * 
 * Discretized using forward Euler integration.
 */
class BicycleModel {
public:
    /**
     * @brief Constructor for the bicycle model
     * 
     * @param dt Simulation time step in seconds
     * @param spec Robot specification containing physical constraints
     */
    BicycleModel(double dt, const RobotSpec& spec);

    /**
     * @brief Compute next state using Euler integration
     * 
     * Performs one simulation step using forward Euler integration:
     * s_{k+1} = s_k + dt * f(s_k, u_k)
     * 
     * @param s Current state [x, y, theta, v]
     * @param u Current control input [steering, acceleration]
     * @return Next state after integration
     */
    State step(const State& s, const Control& u) const;
    
    /**
     * @brief Compute nominal control to reach a goal position
     * 
     * Simple proportional controller that computes steering angle based on
     * the angle error to a goal position, and acceleration based on velocity error.
     * 
     * @param s Current state
     * @param gx Goal x-coordinate
     * @param gy Goal y-coordinate
     * @return Nominal control input [steering, acceleration]
     */
    Control nominal(const State& s, double gx, double gy) const;
    
    /**
     * @brief Compute nominal control to track desired orientation and velocity
     * 
     * Computes control to achieve a desired heading angle and velocity.
     * 
     * @param s Current state
     * @param desired_theta Desired heading angle in radians
     * @param desired_v Desired velocity in m/s
     * @return Nominal control input [steering, acceleration]
     */
    Control nominal_track_ref(const State& s, double desired_theta, double desired_v) const;

    /**
     * @brief Get the robot specification
     * @return Const reference to RobotSpec
     */
    const RobotSpec& spec() const { return spec_; }
    
    /**
     * @brief Get the simulation time step
     * @return Time step in seconds
     */
    double dt() const { return dt_; }

    /**
     * @brief Clamp a value between lower and upper bounds
     * @param x Value to clamp
     * @param lo Lower bound
     * @param hi Upper bound
     * @return Clamped value in [lo, hi]
     */
    static double clamp(double x, double lo, double hi);
    
    /**
     * @brief Wrap an angle to the range [-π, π]
     * @param a Angle in radians to wrap
     * @return Wrapped angle in [-π, π]
     */
    static double wrapAngle(double a);

private:
    double dt_;          ///< Simulation time step
    RobotSpec spec_;     ///< Robot physical specifications
};

} // namespace dpcbf_qp