# DPCBF Algorithm Flow

## DPCBF Implementation Flow

```mermaid
flowchart TD
    Start([Start: solve() method]) --> A[Get current state s and reference control u_ref]
    A --> B[Compute DPCBF h values for all obstacles]
    
    B --> C[Sort obstacles by h value (ascending)]
    C --> D[Select critical obstacles: h < h_thresh]
    D --> E[Add up to K most dangerous obstacles not already selected]
    
    E --> F[Initialize QP matrices H, f, A, l, u]
    F --> G[Set quadratic cost terms for control tracking]
    G --> H[Add jerk penalty terms if previous control available]
    
    H --> I{For each selected obstacle}
    I --> J[Compute discrete DPCBF using reference control]
    J --> K[Compute ∂h/∂steer using finite differences]
    K --> L[Compute ∂h/∂accel using finite differences]
    L --> M[Add linearized DPCBF constraint to matrix A]
    
    M --> N[Add control limits to constraint matrices]
    N --> O[Add slack variable limits]
    O --> P[Setup OSQP solver with matrices]
    P --> Q[Solve QP optimization problem]
    Q --> R{Solution valid?}
    R -->|Yes| S[Apply control limits to solution]
    R -->|No| T[Return reference control]
    T --> U([Return: safe control])
    S --> V[Update previous control for jerk penalty]
    V --> U
    I --> I2[Next obstacle]
    I2 --> J
    M --> N2[Next obstacle constraint]
    N2 --> N