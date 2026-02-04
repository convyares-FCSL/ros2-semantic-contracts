/// @file backend.hpp
/// @brief Internal backend implementation header (not part of public API).
#ifndef BACKEND_PROD_SRC_BACKEND_HPP_
#define BACKEND_PROD_SRC_BACKEND_HPP_

#include "internal/types.hpp"
#include "internal/errors.hpp"
#include "utils/trace.hpp"
#include <unordered_map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <test_msgs/action/fibonacci.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <vector>

namespace backend_prod {

using FibAction = test_msgs::action::Fibonacci;

/// Backend state tracking (goal terminal status, etc.)
struct BackendState {
    // ROS 2 Infrastructure
    rclcpp::Node::SharedPtr node; // Client node
    rclcpp::Node::SharedPtr server_node; // Server (Rig) node
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
    std::thread spinner_thread;
    
    // Action Logic
    rclcpp_action::Client<FibAction>::SharedPtr client;
    rclcpp_action::Server<FibAction>::SharedPtr server;

    // Server Policy (Rig)
    // 0 = Accept, 1 = Reject
    std::atomic<int> next_goal_policy{0};

    // Mapping from ROS 2 UUID (16 bytes) to Scenario Goal ID (string)
    // We use vector<uint8_t> or just string for key? rclcpp_action::GoalUUID is std::array<uint8_t, 16>.
    // std::map doesn't support array key easily without comparator, but we can use generic comparator or cheat.
    // Let's use a simple vector or string key.
    std::mutex map_mutex;
    std::map<std::vector<uint8_t>, std::string> uuid_map;

    /// goal_id -> terminal_status (e.g. "SUCCEEDED")
    std::unordered_map<std::string, std::string> terminal;
    /// goal_id -> terminal set attempts count
    std::unordered_map<std::string, uint32_t> terminal_attempts;

    // Destructor to ensure proper shutdown
    ~BackendState() {
        if (executor) {
            executor->cancel();
        }
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }
    }
};

// Op execution dispatch (called from backend.cpp)
void exec_op(TraceWriter& w, BackendState& st, const std::string& scenario_id, const std::string& op_id, const Op& op);

} // namespace backend_prod

#endif // BACKEND_PROD_SRC_BACKEND_HPP_
