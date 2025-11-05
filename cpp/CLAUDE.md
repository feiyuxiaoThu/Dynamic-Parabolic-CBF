# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

The project uses CMake with C++17 standard. Build with:

```bash
mkdir build && cd build
cmake ..
make -j4
```

Dependencies: Eigen3, OSQP-Eigen, gflags, yaml-cpp

Install dependencies:
```bash
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libosqp-dev libeigen3-dev libyaml-cpp-dev libgflags-dev
```

## Executables

- `unified_examples` - Main simulation program
- `test_multiple_shooting` - Multiple-shooting controller tests
- `benchmark_controllers` - Performance benchmarking
- `test_all_obstacles` - Obstacle testing

## Core Architecture

### UnifiedSimulator Class
The central simulation engine located in `include/unified_simulator.h` and `src/unified_simulator.cpp`. It provides:

- **Controller Management**: Supports single-shooting, multiple-shooting, and adaptive controller selection
- **Scene Configuration**: Handles different scenario types (straight line, intersection)
- **Performance Monitoring**: Tracks controller usage, solve times, and success rates
- **Trajectory Management**: Creates reference trajectories and return trajectories after obstacle avoidance

### Controller Types
1. **Single-Shooting** (`qp_controller.h/cpp`): Fast QP-based controller for simple scenarios
2. **Multiple-Shooting** (`multiple_shooting_controller.h/cpp`): Advanced controller with multi-step prediction for complex environments
3. **Adaptive**: Automatically selects optimal controller based on environment complexity

### Key Components
- **BicycleModel** (`bicycle_model.h`): Vehicle dynamics model
- **DPCBF** (`dpcbf.h`): Dynamic Parabolic Control Barrier Function implementation
- **SplineTrajectory** (`SplineTrajectory.hpp`): Quintic spline trajectory generation
- **AdvancedQPSolver** (`advanced_qp_solver.h/cpp`): Enhanced QP solver for multiple-shooting

### Data Flow
1. Load configuration via gflags
2. Create waypoints and obstacles for scenario
3. Generate reference trajectory using splines
4. Run simulation loop with adaptive controller selection
5. Optionally save results to CSV
6. Visualize results using Python scripts

## Running Simulations

### Basic Usage
```bash
# Straight line scenario
./unified_examples --scenario straight

# Intersection scenario
./unified_examples --scenario intersection

# Both scenarios
./unified_examples --scenario both

# Disable CSV output for online use
./unified_examples --scenario straight --save_csv=false
```

### Advanced Usage
```bash
# Test multiple-shooting controller
./test_multiple_shooting --controller=multiple --scenario=straight --ms_horizon=15

# Run performance benchmarks
./benchmark_controllers

# Adaptive controller testing
./test_multiple_shooting --controller=adaptive --scenario=both --verbose
```

## Visualization

Python visualization scripts in `viz/` directory:
- `unified_visualizer.py` - Main visualization for both scenarios
- `ms_visualizer.py` - Multiple-shooting specific visualization
- `enhanced_visualizer.py` - Enhanced visualization features

Run visualization:
```bash
python3 viz/unified_visualizer.py --scenario straight
python3 viz/unified_visualizer.py --scenario intersection
```

## Configuration

### Key Parameters
- **Robot specs**: radius (0.3m), max acceleration (5.0 m/s²), max steering (0.5 rad)
- **DPCBF**: margin (2.2m), k_lambda (8.0), k_mu (0.05)
- **Simulation**: dt (0.05s), v_ref (6.0 m/s)
- **Multiple-Shooting**: horizon (10), weight_control (12.0), max_obstacles (3)

### Adaptive Controller Selection
The system automatically selects controllers based on:
- Environment complexity (obstacle density and relative velocities)
- Time budget constraints (default 20ms)
- Performance statistics from previous solves

## File Organization

```
cpp/
├── include/           # Header files
├── src/              # Source implementations
├── viz/              # Python visualization scripts
├── build/            # Build directory
├── unified_examples.cpp    # Main executable
├── test_*.cpp        # Test programs
└── benchmark_*.cpp   # Benchmark programs
```

## Important Implementation Notes

### Trajectory Extension Strategy
The system extends reference trajectories beyond the endpoint by 3 seconds to ensure sufficient reference information for return trajectory calculations near the end. This extension is used only for reference, not for actual vehicle movement.

### CSV Output Format
When enabled, CSV files include: time, position, velocity, control inputs, reference trajectory information, obstacle states, and safety distances.

### Performance Monitoring
The UnifiedSimulator tracks controller performance metrics including usage count, solve times, success rates, and max/min solve times for controller selection optimization.