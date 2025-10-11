#pragma once
#include "bicycle_model.h"
#include "dpcbf.h"
#include "qp_controller.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

struct Waypoint {
    double x, y, theta;
};

struct SimConfig {
    double dt;
    double v_ref;
    RobotSpec robot_spec;
    std::vector<Waypoint> waypoints;
    std::vector<Obstacle> obstacles;
    DPCBFParams dpcbf_params;
    QPWeights qp_weights;
    DiscreteCBFConfig discrete_cbf_config;
};

class ConfigLoader {
public:
    static SimConfig loadConfig(const std::string& filename);
};