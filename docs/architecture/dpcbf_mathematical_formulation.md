# Dynamic Parabolic Control Barrier Function (DPCBF) Mathematical Formulation

## 1. Introduction

Dynamic Parabolic Control Barrier Functions (DPCBFs) are a class of time-varying Control Barrier Functions (CBFs) that adapt their safety constraints based on the relative distance and velocity to obstacles. Unlike traditional CBFs with fixed safety sets, DPCBFs define a parabolic safety boundary that changes shape dynamically.

## 2. Mathematical Definition

### 2.1 Continuous Time DPCBF

For a system with state x, the continuous-time DPCBF is defined as:

```
h(x) = v_x + λ(x) * v_y² + μ(x)
```

Where:
- `v_x` is the relative velocity component along the Line-of-Sight (LOS) direction (radial velocity)
- `v_y` is the relative velocity component perpendicular to the LOS direction (tangential velocity)
- `λ(x)` is the adaptive curvature parameter
- `μ(x)` is the adaptive offset parameter

### 2.2 Dynamic Parameters

The parameters `λ(x)` and `μ(x)` are computed dynamically as:

```
scale = √(margin² - 1) / (ego_dim + ε)
λ(x) = (k_λ * scale) * √(d_safe) / ||v_rel|| 
μ(x) = (k_μ * scale) * √(d_safe)
```

Where:
- `margin` is the safety margin coefficient (≥ 1)
- `ego_dim = (r_obs + r_robot) * margin` - effective collision radius with safety margin
- `d_safe = max(||p_rel||² - ego_dim², ε)` - safety distance
- `k_λ, k_μ` are design parameters
- `ε` is a small positive constant for numerical stability

### 2.3 Relative Kinematics

The relative kinematics between robot and obstacle are computed as:

```
p_rel = [ox - x, oy - y]           (relative position)
v_rel = [v_ox - v*cos(θ), v_oy - v*sin(θ)]  (relative velocity)
```

The LOS transformation rotates the coordinate system so that the x-axis points from robot to obstacle:

```
R(α) = [cos(α)   sin(α)]
       [-sin(α)  cos(α)]

v_x = R(α) * v_rel_x
v_y = R(α) * v_rel_y
```

## 3. Discrete Time Implementation

For discrete-time control with sampling period T, the DPCBF constraint is formulated as:

```
h_{k+1} - h_k + γ*h_k ≥ 0
```

Where:
- `h_k = h(x_k)` is the DPCBF value at current time
- `h_{k+1} = h(x_{k+1})` is the predicted DPCBF value
- `γ > 0` is the discrete decay rate parameter

This ensures forward invariance of the safe set in discrete time.

## 4. Linearization for QP Formulation

The discrete DPCBF constraint is linearized around the reference control `u_ref`:

```
∇h * δu ≥ -c
```

Where:
- `δu = u - u_ref` is the control deviation from reference
- `∇h` is the gradient of the constraint with respect to control inputs
- `c` incorporates the constant terms from linearization

The gradient is computed using finite differences:

```
∂h/∂steer ≈ (h(u_ref + [Δ, 0]) - h(u_ref)) / Δ
∂h/∂accel ≈ (h(u_ref + [0, Δ]) - h(u_ref)) / Δ
```

## 5. CBF-QP Formulation

The complete CBF-QP is formulated as:

```
minimize: ||u - u_ref||_W² + ||u - u_prev||_W_jerk² + ρ*s²
subject to:
  ∇h_i * (u - u_ref) ≥ -c_i - s    ∀i ∈ selected_obstacles
  u_min ≤ u ≤ u_max
  0 ≤ s ≤ s_max
```

Where:
- `||·||_W²` denotes weighted quadratic norm
- `u_ref` is the nominal (unsafe) control from the path tracking controller
- `u_prev` is the previous control input for jerk minimization
- `s` is the slack variable for constraint relaxation
- `s_max` is the maximum allowed constraint violation

## 6. Adaptive Properties

The DPCBF formulation has the following adaptive characteristics:

### 6.1 Distance Adaptivity
- As `||p_rel||` increases → `d_safe` increases → `λ` and `μ` increase
- Larger safety region when far from obstacles (more maneuvering space)

### 6.2 Velocity Adaptivity
- As `||v_rel||` increases → `λ` decreases → parabola becomes more open
- Allows more lateral movement during high relative velocity encounters

### 6.3 Safety Margin Adaptivity
- Larger `margin` → larger `ego_dim` → more conservative behavior
- Provides configurable safety levels

## 7. Design Parameters

### 7.1 DPCBF Parameters
- `k_λ`: Scales the tangential velocity penalty (default: 0.1)
  - Higher values → stronger penalty on lateral movement
- `k_μ`: Scales the distance-based safety margin (default: 0.5)
  - Higher values → larger safety margins
- `margin`: Safety coefficient (default: 1.05)
  - Should be > 1.0 for safety buffer

### 7.2 Discrete CBF Parameters
- `γ`: Discrete decay rate (default: 0.25)
  - Higher values → stronger safety guarantees but more conservative
- `Δ`: Finite difference step (default: 1e-3)
  - Numerical accuracy vs. conditioning trade-off

### 7.3 QP Weights
- `w_steer, w_accel`: Tracking weights (default: 12.0)
  - Balance between safety and performance
- `w_jerk_steer, w_jerk_accel`: Jerk penalty weights
  - Smooth control transitions
- `ρ`: Slack variable weight
  - Balance between feasibility and safety

## 8. Advantages over Traditional CBFs

1. **Adaptive Safety Boundaries**: The parabolic boundary adapts based on situation
2. **Reduced Conservatism**: Allows more maneuvering space when safe
3. **Time-Varying Safe Sets**: Better suited for dynamic environments
4. **Intuitive Parameters**: Physical meaning of parameters is clear
5. **Computational Efficiency**: Suitable for real-time applications

## 9. Implementation Considerations

### 9.1 Numerical Stability
- Use `ε` terms to avoid division by zero
- Clamp values to prevent numerical overflow
- Verify positive definiteness of QP Hessian

### 9.2 Constraint Selection
- Only include most critical obstacles to limit QP size
- Use threshold-based selection combined with top-K selection
- Regular re-evaluation of obstacle priorities

### 9.3 Multi-Obstacle Handling
- Potential for constraint conflicts with multiple obstacles
- Slack variable provides feasibility guarantee
- Hierarchical approaches possible for critical conflicts