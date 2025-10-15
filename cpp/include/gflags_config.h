#pragma once
#include "bicycle_model.h"
#include "dpcbf.h"
#include "qp_controller.h"
#include <gflags/gflags.h>
#include <vector>

namespace dpcbf_qp {

struct Waypoint {
    double x, y, theta;
};

struct SimConfig {
    double dt;
    double v_ref;
    RobotSpec robot_spec;
    DPCBFParams dpcbf_params;
    QPWeights qp_weights;
    DiscreteCBFConfig discrete_cbf_config;
};

class GFlagsConfig {
public:
    static SimConfig loadConfig();
    static void defineFlags();
};

} // namespace dpcbf_qp