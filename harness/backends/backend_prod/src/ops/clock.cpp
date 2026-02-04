/// @file clock.cpp
/// @brief Clock-related operations that exercise real ROS 2 time APIs.
#include "../backend.hpp"
#include "internal/errors.hpp"
#include <rclcpp/rclcpp.hpp>

namespace backend_prod::ops {

/// Executes get_clock: creates a temporary node and reads the current ROS time.
/// This exercises real rclcpp node creation and clock APIs.
/// Result is returned in op_end.detail with timestamp_ns.
json exec_get_clock(TraceWriter& /*w*/, const std::string& /*scenario_id*/) {
    // Create a temporary node with unique name to avoid conflicts
    static int node_counter = 0;
    std::string node_name = "oracle_clock_node_" + std::to_string(node_counter++);

    auto node = std::make_shared<rclcpp::Node>(node_name);

    // Get the current ROS time using the node's clock
    rclcpp::Time now = node->get_clock()->now();
    int64_t timestamp_ns = now.nanoseconds();

    // Return the result to be included in op_end.detail
    return json{
        {"op", "get_clock"},
        {"timestamp_ns", timestamp_ns},
        {"clock_type", "ros_time"}
    };
}

} // namespace backend_prod::ops
