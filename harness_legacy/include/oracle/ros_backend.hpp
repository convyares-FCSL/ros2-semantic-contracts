#pragma once
#include "oracle/oracle_core.hpp"

namespace oracle {

// ROS-backed execution (implementation in src/ros_backend.cpp).
int run_oracle_ros(const RunConfig& cfg);

}  // namespace oracle
