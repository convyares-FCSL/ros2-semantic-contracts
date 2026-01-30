#pragma once

#include "oracle/oracle_core.hpp"

namespace oracle {

// Runs mechanistic scenarios via real ROS2 rclcpp action interactions.
// Currently implements A01 only; other scenarios will be added incrementally.
int run_oracle_ros(const RunConfig& cfg);

}  // namespace oracle
