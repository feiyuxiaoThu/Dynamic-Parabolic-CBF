#include "../include/gflags_config.h"
#include <gflags/gflags.h>

// Simulation parameters
DEFINE_double(dt, 0.05, "Simulation time step");
DEFINE_double(v_ref, 5.0, "Reference velocity");

// Robot specification
DEFINE_double(robot_radius, 2.5, "Robot radius");
DEFINE_double(robot_a_max, 5.0, "Maximum acceleration");
DEFINE_double(robot_steer_max, 0.5, "Maximum steering angle");
DEFINE_double(robot_L, 3.0, "Robot wheelbase length");

// DPCBF parameters
DEFINE_double(dpcbf_k_lambda, 0.50, "DPCBF k_lambda parameter");
DEFINE_double(dpcbf_k_mu, 0.1, "DPCBF k_mu parameter");
DEFINE_double(dpcbf_margin, 2.20, "DPCBF margin parameter");
DEFINE_double(dpcbf_eps, 1.0e-6, "DPCBF epsilon parameter");

// QP weights
DEFINE_double(qp_w_steer, 100.0, "QP steering weight");
DEFINE_double(qp_w_a, 20.0, "QP acceleration weight");
DEFINE_double(qp_rho, 20.0, "QP rho parameter");

// Discrete CBF config
DEFINE_double(discrete_cbf_gamma, 0.22, "Discrete CBF gamma parameter");
DEFINE_double(discrete_cbf_du, 1.0e-3, "Discrete CBF du parameter");
DEFINE_double(discrete_cbf_s_max, 10.0, "Discrete CBF s_max parameter");

namespace dpcbf_qp {

void GFlagsConfig::defineFlags() {
    // Flags are already defined above using DEFINE_* macros
}

SimConfig GFlagsConfig::loadConfig() {
    SimConfig config;
    
    // Simulation parameters
    config.dt = FLAGS_dt;
    config.v_ref = FLAGS_v_ref;
    
    // Robot specification
    config.robot_spec.radius = FLAGS_robot_radius;
    config.robot_spec.a_max = FLAGS_robot_a_max;
    config.robot_spec.steer_max = FLAGS_robot_steer_max;
    config.robot_spec.L = FLAGS_robot_L;
    
    // DPCBF parameters
    config.dpcbf_params.k_lambda = FLAGS_dpcbf_k_lambda;
    config.dpcbf_params.k_mu = FLAGS_dpcbf_k_mu;
    config.dpcbf_params.margin = FLAGS_dpcbf_margin;
    config.dpcbf_params.eps = FLAGS_dpcbf_eps;
    
    // QP weights
    config.qp_weights.w_steer = FLAGS_qp_w_steer;
    config.qp_weights.w_a = FLAGS_qp_w_a;
    config.qp_weights.rho = FLAGS_qp_rho;
    
    // Discrete CBF config
    config.discrete_cbf_config.gamma = FLAGS_discrete_cbf_gamma;
    config.discrete_cbf_config.du = FLAGS_discrete_cbf_du;
    config.discrete_cbf_config.s_max = FLAGS_discrete_cbf_s_max;
    
    return config;
}

} // namespace dpcbf_qp