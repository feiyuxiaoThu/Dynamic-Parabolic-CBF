#include <iostream>
#include <vector>
#include "include/qp_controller.h"
#include "include/bicycle_model.h"
#include "include/dpcbf.h"

using namespace dpcbf_qp;

int main() {
    // Create robot specification
    RobotSpec robot_spec;
    robot_spec.radius = 0.5;
    robot_spec.a_max = 5.0;
    robot_spec.steer_max = 0.5;
    robot_spec.L = 2.0;
    
    // Create bicycle model
    BicycleModel model(0.05, robot_spec);
    
    // Create DPCBF parameters
    DPCBFParams dparams;
    dparams.k_lambda = 0.1;
    dparams.k_mu = 0.5;
    dparams.margin = 1.05;
    dparams.eps = 1e-6;
    
    // Create QP weights
    QPWeights weights;
    weights.w_steer = 12.0;
    weights.w_a = 12.0;
    weights.w_jerk_steer = 5.0;
    weights.w_jerk_accel = 200.0;
    weights.rho = 20.0;
    
    // Create discrete CBF config for ALL obstacles
    DiscreteCBFConfig cfg;
    cfg.gamma = 0.25;
    cfg.du = 1e-3;
    cfg.s_max = 10.0;
    cfg.consider_all_obstacles = true;  // Enable all obstacles mode
    cfg.h_threshold = 0.2;
    cfg.max_obstacles = 100;  // Large number to allow many obstacles if needed
    
    // Create QP controller
    QPController controller(model, dparams, weights, cfg);
    
    // Test state
    State s;
    s.x = 0.0;
    s.y = 0.0;
    s.theta = 0.0;
    s.v = 5.0;
    
    // Reference control
    Control u_ref;
    u_ref.steer = 0.0;
    u_ref.a = 0.0;
    
    // Create multiple obstacles to test "all obstacles" functionality
    std::vector<Obstacle> obstacles = {
        {10.0, 2.0, 1.0, -1.0, 0.0},   // Obstacle 1
        {15.0, -1.0, 1.5, -2.0, 0.5},  // Obstacle 2
        {20.0, 3.0, 0.8, -1.5, -0.5},  // Obstacle 3
        {25.0, -2.0, 1.2, -2.5, 0.0},  // Obstacle 4
        {30.0, 1.0, 1.0, -1.0, 1.0}    // Obstacle 5
    };
    
    std::cout << "Number of obstacles: " << obstacles.size() << std::endl;
    std::cout << "consider_all_obstacles mode: " << (cfg.consider_all_obstacles ? "ON" : "OFF") << std::endl;
    
    // Solve with all obstacles
    Control result = controller.solve(s, u_ref, obstacles);
    
    std::cout << "Control result:" << std::endl;
    std::cout << "  Steering: " << result.steer << std::endl;
    std::cout << "  Acceleration: " << result.a << std::endl;
    
    std::cout << "Test completed successfully!" << std::endl;
    
    return 0;
}