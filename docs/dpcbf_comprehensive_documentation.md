# Dynamic Parabolic Control Barrier Functions (DPCBF) - Comprehensive Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Mathematical Foundation](#mathematical-foundation)
3. [System Architecture](#system-architecture)
4. [Implementation Details](#implementation-details)
5. [Code Structure](#code-structure)
6. [Key Algorithms](#key-algorithms)
7. [Configuration and Parameters](#configuration-and-parameters)
8. [Simulation Scenarios](#simulation-scenarios)
9. [Performance and Optimization](#performance-and-optimization)
10. [Comparison with Other Methods](#comparison-with-other-methods)

## Introduction

Dynamic Parabolic Control Barrier Functions (DPCBF) represent an advanced approach to safety-critical control for autonomous systems operating in dynamic environments. Unlike traditional Control Barrier Functions (CBFs) with fixed safety sets, DPCBFs define adaptive, time-varying safety boundaries that change based on the relative distance and velocity to obstacles.

This implementation provides a complete framework for applying DPCBFs to autonomous vehicle navigation, featuring:
- Real-time capable CBF-QP formulation
- Adaptive safety boundaries that adjust to dynamic conditions
- Integration with kinematic bicycle model for vehicle dynamics
- Comprehensive simulation environment with multiple scenarios

## Mathematical Foundation

### The DPCBF Formulation

The DPCBF defines a time-varying safe set using a parabolic boundary in the relative velocity space. For a system with state x and an obstacle with state o, the DPCBF is defined as:

```math
h(x, o) = v_x + λ(x, o) \cdot v_y^2 + μ(x, o) ≥ 0
```

Where:
- `v_x`: Relative velocity component along the Line-of-Sight (LOS) direction (radial velocity)
- `v_y`: Relative velocity component perpendicular to the LOS direction (tangential velocity)  
- `λ(x, o)`: Adaptive curvature parameter for the parabolic boundary
- `μ(x, o)`: Adaptive offset parameter for safety margin

### Dynamic Parameter Computation

The parameters adapt based on geometric and kinematic properties:

```math
scale = \frac{\sqrt{margin^2 - 1}}{ego\_dim + ε}
```

```math
λ(x, o) = (k_λ \cdot scale) \cdot \frac{\sqrt{d_{safe}}}{||v_{rel}|| + ε}
```

```math
μ(x, o) = (k_μ \cdot scale) \cdot \sqrt{d_{safe}}
```

Where:
- `ego_dim = (r_{obs} + r_{robot}) \cdot margin` - effective collision radius
- `d_{safe} = max(||p_{rel}||^2 - ego\_dim^2, ε)` - safety distance measure
- `k_λ, k_μ` - design parameters controlling behavior
- `margin` - safety margin coefficient (≥ 1.0)

### Adaptive Properties

1. **Distance Adaptivity**: As distance to obstacle increases, both λ and μ increase, allowing more maneuvering space
2. **Velocity Adaptivity**: As relative velocity increases, λ decreases, allowing more lateral movement
3. **Safety Margin Control**: margin parameter provides configurable safety levels

## System Architecture

### Component Overview

The DPCBF system consists of several interconnected components:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Main Control   │────│ Unified         │────│  Trajectory     │
│  Application    │    │  Simulator      │    │  Generation     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                             │                         │
                    ┌────────▼────────┐        ┌───────▼────────┐
                    │ QP Controller   │        │ Spline        │
                    │ (CBF-QP)        │        │ Trajectories  │
                    └─────────────────┘        └───────────────┘
                             │
                    ┌────────▼────────┐
                    │ DPCBF           │
                    │ Functions       │
                    └─────────────────┘
                             │
                    ┌────────▼────────┐
                    │ Bicycle Model   │
                    │ (Dynamics)      │
                    └─────────────────┘
```

### Key Components

1. **Unified Simulator**: Orchestrates the entire simulation pipeline, managing trajectories, obstacles, and control flow
2. **QP Controller**: Implements the CBF-QP optimization for safety-critical control
3. **DPCBF Functions**: Provides the mathematical core for computing barrier functions
4. **Bicycle Model**: Handles vehicle kinematics and state propagation
5. **Spline Trajectories**: Manages reference and return trajectory generation

## Implementation Details

### Core DPCBF Implementation

The core DPCBF functions are implemented in `dpcbf.h` and `dpcbf.cc`:

#### dpcbf_continuous()
Computes the continuous-time DPCBF value:

```cpp
DPCBFResult dpcbf_continuous(double X, double Y, double theta, double v,
                             const Obstacle& obs, double robot_radius,
                             const DPCBFParams& p)
```

Steps:
1. Compute relative position and velocity between robot and obstacle
2. Transform velocity to Line-of-Sight coordinate system
3. Compute dynamic parameters λ and μ based on distance and relative velocity
4. Evaluate the parabolic barrier function: `h = v_x + λ*v_y² + μ`

#### dpcbf_discrete()
Computes discrete-time values for QP constraint formation:

```cpp
std::array<double,2> dpcbf_discrete(double X, double Y, double theta, double v,
                                    double X1, double Y1, double theta1, double v1,
                                    const Obstacle& obs, double robot_radius,
                                    const DPCBFParams& p)
```

Returns: `{h_current, h_next - h_current}` for discrete constraint formation.

### QP Controller Implementation

The QP controller in `qp_controller.h` and `qp_controller.cc` implements the safety-critical optimization:

#### Core Algorithm
1. **Obstacle Selection**: Selects most critical obstacles based on DPCBF values
2. **Constraint Formation**: Linearizes DPCBF constraints for selected obstacles
3. **QP Optimization**: Solves the constrained optimization problem
4. **Control Output**: Returns safe control input that balances performance and safety

#### Objective Function
```math
\min_{u,s} ||u - u_{ref}||_W^2 + ||u - u_{prev}||_{W_{jerk}}^2 + ρ \cdot s^2
```

Balances:
- Tracking reference control (u_ref)
- Smooth control transitions (jerk penalty)
- Constraint satisfaction (slack penalty)

## Code Structure

### Header Files
- `dpcbf.h`: Core DPCBF data structures and function declarations
- `qp_controller.h`: QP controller interface and configuration
- `bicycle_model.h`: Vehicle dynamics and state representation
- `unified_simulator.h`: High-level simulation interface
- `SplineTrajectory.hpp`: Trajectory generation and interpolation

### Source Files
- `dpcbf.cc`: Implementation of DPCBF mathematical functions
- `qp_controller.cc`: CBF-QP formulation and optimization
- `bicycle_model.cc`: Vehicle kinematics and nominal control
- `unified_simulator.cpp`: Simulation orchestration and scenario management

### Key Data Structures

#### State
Represents vehicle state: `[x, y, θ, v]` (position, orientation, velocity)

#### Control
Represents control input: `[steer, a]` (steering angle, acceleration)

#### Obstacle
Represents dynamic obstacle: `[ox, oy, r, vx, vy]` (position, radius, velocity)

#### DPCBFParams
Configuration: `[k_lambda, k_mu, margin, eps]` (adaptive parameters)

## Key Algorithms

### 1. Line-of-Sight (LOS) Transformation
Converts from global coordinates to obstacle-relative coordinates where the x-axis points from robot to obstacle:

```cpp
void rotateToLOS(double px, double py, double vx, double vy, double& vxl, double& vyl) {
    double rot = atan2(py, px);  // Angle from robot to obstacle
    double c = cos(rot), s = sin(rot);
    vxl = c*vx + s*vy;  // Radial component
    vyl = -s*vx + c*vy; // Tangential component
}
```

### 2. Adaptive Parameter Computation
Computes the dynamic parameters that make DPCBF adaptive:

```cpp
double scale = sqrt(p.margin*p.margin - 1.0) / (ego_dim + p.eps);
double lam = (p.k_lambda * scale) * sqrt(dsafe) / (vr + p.eps);
double mu  = (p.k_mu * scale) * sqrt(dsafe);
```

### 3. Obstacle Selection Strategy
Selects obstacles for constraint inclusion using dual criteria:

```cpp
// First, select all critical obstacles (h < threshold)
for (const auto& pr : scores) {
    if (pr.first < h_thresh) sel.push_back(pr.second);
}
// Then, add the K most dangerous if needed
```

### 4. Numerical Gradient Computation
Computes constraint gradients for QP formulation:

```cpp
double J_steer = (c_steer - c0) / cfg_.du;  // ∂(constraint)/∂(steering)
double J_a     = (c_a - c0) / cfg_.du;     // ∂(constraint)/∂(acceleration)
```

## Configuration and Parameters

### DPCBF Parameters
- `k_lambda`: Controls tangential velocity penalty (default: 0.1)
- `k_mu`: Controls distance-based safety margin (default: 0.5) 
- `margin`: Safety coefficient for collision radius (default: 1.05)
- `eps`: Numerical stability constant (default: 1e-6)

### QP Weights
- `w_steer`: Steering tracking weight (default: 12.0)
- `w_a`: Acceleration tracking weight (default: 12.0)
- `w_jerk_steer`: Steering jerk penalty (default: 5.0)
- `w_jerk_accel`: Acceleration jerk penalty (default: 200.0)
- `rho`: Slack variable penalty (default: 20.0)

### Discrete CBF Configuration
- `gamma`: Discrete decay rate (default: 0.25)
- `du`: Numerical gradient step (default: 1e-3)
- `s_max`: Maximum slack (default: 10.0)

## Simulation Scenarios

### Straight Line Scenario
- **Setup**: Highway-like environment with multiple dynamic obstacles
- **Trajectory**: Linear path from (1, 7.5) to (50, 7.5)
- **Challenges**: Multi-vehicle interactions, high-speed encounters

### Intersection Scenario  
- **Setup**: Right-turn intersection with complex obstacle patterns
- **Trajectory**: Quarter-circle right turn
- **Challenges**: Turning maneuvers, cross-traffic interactions

### Key Features
- **Trajectory Extension**: Reference trajectory extended beyond endpoint for return calculations
- **Return Trajectory**: Automatically computed to get back to reference after avoidance
- **Obstacle Prediction**: Dynamic obstacle motion prediction integrated

## Performance and Optimization

### Computational Efficiency
- **Obstacle Selection**: Limits constraints to most critical obstacles (≤ 3)
- **Numerical Optimization**: Efficient gradient computation and QP formulation  
- **Caching**: Previous control values cached for jerk computation

### Numerical Stability
- **Epsilon Terms**: Prevent division by zero and numerical issues
- **Control Limits**: Physical constraints ensure realistic control inputs
- **Clamping**: Output values clamped to feasible ranges

### Memory Management
- **Pre-allocation**: Vector capacities reserved to avoid reallocation
- **Efficient Structures**: Eigen matrices used for linear algebra operations
- **Reused Objects**: Temporary objects reused across iterations

## Comparison with Other Methods

### vs. Traditional CBF
- **Adaptivity**: DPCBF adapts to dynamic conditions, traditional CBF has fixed safety sets
- **Conservatism**: DPCBF provides more maneuvering space when safe
- **Complexity**: DPCBF has more complex parameter tuning but better performance

### vs. Collision Cone CBF (C3BF)
- **Geometry**: DPCBF uses parabolic boundaries, C3BF uses conic boundaries  
- **Flexibility**: DPCBF adapts shape based on kinematics, C3BF has fixed angle
- **Parameters**: DPCBF parameters have clearer physical interpretation

### vs. Pure QP Methods
- **Safety Guarantees**: DPCBF provides formal safety guarantees, pure QP does not
- **Constraint Formulation**: DPCBF uses mathematically principled constraints
- **Performance**: Balance of safety and performance is mathematically justified

## Conclusion

The DPCBF implementation provides a robust, mathematically principled approach to safety-critical control in dynamic environments. Its adaptive nature and formal guarantees make it suitable for real-world autonomous navigation applications while maintaining computational efficiency for real-time operation.