# Dynamic Parabolic CBF System Architecture

## Overview
This document describes the architecture of the Dynamic Parabolic Control Barrier Function (DPCBF) system for safe navigation of autonomous vehicles in dynamic environments.

## System Architecture Diagram

```mermaid
graph TB
    subgraph "User Interface"
        A[Main Application] --> B[UnifiedSimulator]
    end
    
    subgraph "Core System Components"
        B --> C[BicycleModel]
        B --> D[QPController]
        B --> E[SplineTrajectory]
        D --> F[DPCBF]
    end
    
    subgraph "Input/Output"
        G[Waypoints] --> B
        H[Obstacles] --> B
        I[Vehicle State] --> D
        J[Control Reference] --> D
        K[Simulation Data] --> L[CSV Output]
        D --> M[Control Output]
    end
    
    subgraph "DPCBF Core Functions"
        F --> F1[dpcbf_continuous]
        F --> F2[dpcbf_discrete]
        F1 --> F3[Line-of-Sight Transformation]
        F2 --> F4[Discrete Constraint Formation]
    end
    
    subgraph "QP Controller Core"
        D --> D1[Obstacle Selection]
        D --> D2[Constraint Linearization]
        D --> D3[QP Optimization]
        D2 --> D4[Numerical Gradient]
        D3 --> D5[OSQP Solver]
    end
    
    subgraph "Trajectory Management"
        B --> E1[Reference Trajectory]
        B --> E2[Return Trajectory]
        B --> E3[Trajectory Extension]
        E1 --> E[Quintic Spline]
        E2 --> E
        E3 --> E
    end
    
    style A fill:#e1f5fe
    style F fill:#f3e5f5
    style D fill:#e8f5e8
    style C fill:#fff3e0
    style E fill:#fce4ec
```

## Component Interaction Flow

```mermaid
sequenceDiagram
    participant U as UnifiedSimulator
    participant BM as BicycleModel
    participant QP as QPController
    participant DPCBF as DPCBF
    participant OSQP as OSQP Solver
    participant S as SplineTrajectory
    
    U->>S: createReferenceTrajectory(waypoints)
    U->>S: computeReturnTrajectory(state, ref_spline)
    U->>S: computeFutureControl(return_spline)
    
    loop Simulation Step
        U->>QP: solve(state, control_ref, obstacles)
        
        QP->>DPCBF: dpcbf_continuous() for all obstacles
        DPCBF-->>QP: h values
        QP->>QP: select critical obstacles
        
        QP->>BM: step(state, control_ref) for linearization
        BM-->>QP: predicted_state
        
        QP->>DPCBF: dpcbf_discrete() for constraints
        DPCBF-->>QP: discrete h values
        
        QP->>QP: compute constraint gradients (numerical)
        QP->>OSQP: setup and solve QP
        OSQP-->>QP: control solution
        
        QP-->>U: safe_control
        U->>BM: step(state, safe_control)
        BM-->>U: new_state
    end
```

## Key Design Patterns

### 1. Component-Based Architecture
The system is organized into modular components that handle specific responsibilities:
- **BicycleModel**: Handles vehicle kinematics and basic control functions
- **DPCBF**: Implements the Dynamic Parabolic Control Barrier Function logic
- **QPController**: Manages the safety-critical optimization problem
- **SplineTrajectory**: Handles trajectory generation and interpolation
- **UnifiedSimulator**: Coordinates all components for the simulation

### 2. Observer Pattern
The system uses parameter structures to configure components:
- `DPCBFParams`: Configures DPCBF behavior (k_lambda, k_mu, margin)
- `QPWeights`: Configures QP objective (tracking, jerk penalties)
- `DiscreteCBFConfig`: Configures discrete constraint parameters

### 3. Strategy Pattern
Different simulation scenarios use different configuration strategies:
- Straight line scenario with highway-like obstacle patterns
- Intersection scenario with complex maneuvering requirements

## DPCBF Mathematical Foundation

The Dynamic Parabolic CBF (DPCBF) defines a time-varying safe set based on:
`h(x) = v_x + λ(x) * v_y² + μ(x) ≥ 0`

Where:
- `v_x`: Relative velocity in the line-of-sight direction
- `v_y`: Relative velocity perpendicular to line-of-sight
- `λ(x)`: Adaptive curvature parameter that changes with distance and relative velocity
- `μ(x)`: Adaptive safety margin parameter

The parameters λ and μ adapt dynamically:
- As distance to obstacle increases → parameters increase (allows more maneuvering)
- As relative velocity increases → λ decreases (allows more lateral movement)

## Safety Architecture

The safety architecture implements multiple layers:

1. **Primary Safety**: DPCBF constraints ensure collision avoidance
2. **Constraint Relaxation**: Slack variables ensure QP feasibility
3. **Control Limits**: Physical limits prevent dangerous control inputs
4. **Jerk Limiting**: Smooth transitions prevent actuator damage

## Performance Considerations

- **Real-time Capability**: QP formulation designed for real-time solution
- **Obstacle Selection**: Only critical obstacles included to limit computational complexity
- **Numerical Stability**: Epsilon terms prevent division by zero
- **Caching**: Previous control inputs cached for jerk computation