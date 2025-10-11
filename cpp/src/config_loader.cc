#include "../include/config_loader.h"
#include <iostream>

SimConfig ConfigLoader::loadConfig(const std::string& filename) {
    SimConfig config;
    YAML::Node node = YAML::LoadFile(filename);

    config.dt = node["simulation"]["dt"].as<double>();
    config.v_ref = node["simulation"]["v_ref"].as<double>();

    config.robot_spec.radius = node["robot_spec"]["radius"].as<double>();
    config.robot_spec.a_max = node["robot_spec"]["a_max"].as<double>();
    config.robot_spec.steer_max = node["robot_spec"]["steer_max"].as<double>();
    config.robot_spec.L = node["robot_spec"]["L"].as<double>();

    for (const auto& wp_node : node["waypoints"]) {
        config.waypoints.push_back({
            wp_node["x"].as<double>(),
            wp_node["y"].as<double>(),
            wp_node["theta"].as<double>()
        });
    }

    for (const auto& obs_node : node["obstacles"]) {
        config.obstacles.push_back({
            obs_node["ox"].as<double>(),
            obs_node["oy"].as<double>(),
            obs_node["r"].as<double>(),
            obs_node["vx"].as<double>(),
            obs_node["vy"].as<double>()
        });
    }

    config.dpcbf_params.k_lambda = node["dpcbf_params"]["k_lambda"].as<double>();
    config.dpcbf_params.k_mu = node["dpcbf_params"]["k_mu"].as<double>();
    config.dpcbf_params.margin = node["dpcbf_params"]["margin"].as<double>();
    config.dpcbf_params.eps = node["dpcbf_params"]["eps"].as<double>();

    config.qp_weights.w_steer = node["qp_weights"]["w_steer"].as<double>();
    config.qp_weights.w_a = node["qp_weights"]["w_a"].as<double>();
    config.qp_weights.rho = node["qp_weights"]["rho"].as<double>();

    config.discrete_cbf_config.gamma = node["discrete_cbf_config"]["gamma"].as<double>();
    config.discrete_cbf_config.du = node["discrete_cbf_config"]["du"].as<double>();
    config.discrete_cbf_config.s_max = node["discrete_cbf_config"]["s_max"].as<double>();

    return config;
}