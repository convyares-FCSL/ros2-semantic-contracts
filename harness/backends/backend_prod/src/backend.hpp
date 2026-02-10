/// @file backend.hpp
/// @brief Internal backend implementation header (not part of public API).
#ifndef BACKEND_PROD_SRC_BACKEND_HPP_
#define BACKEND_PROD_SRC_BACKEND_HPP_

#include "internal/types.hpp"
#include <deque>
#include "internal/errors.hpp"
#include "utils/trace.hpp"
#include <unordered_map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <test_msgs/action/fibonacci.hpp>
#include <thread>
#include <mutex>
#include <map>
#include <vector>

namespace backend_prod {

using FibAction = test_msgs::action::Fibonacci;

#if RCLCPP_VERSION_GTE(17, 0, 0)
#define BACKEND_PROD_HAS_PRE_SET_CALLBACK 1
#else
#define BACKEND_PROD_HAS_PRE_SET_CALLBACK 0
#endif

/// Backend state tracking (goal terminal status, etc.)
struct BackendState {
    // ROS 2 Infrastructure
    rclcpp::Node::SharedPtr node; // Client node
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
    std::thread spinner_thread;

    // Action Logic
    rclcpp_action::Client<FibAction>::SharedPtr client;

    // NEW: Status subscription (shared across scenarios)
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub;

    // Parameter callback handles kept alive for scenario lifetime.
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_handle;
#if BACKEND_PROD_HAS_PRE_SET_CALLBACK
    rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr pre_set_params_cb_handle;
#endif

    // Mapping from ROS 2 UUID (16 bytes) to Scenario Goal ID (string)
    // We use vector<uint8_t> or just string for key? rclcpp_action::GoalUUID is std::array<uint8_t, 16>.
    // std::map doesn't support array key easily without comparator, but we can use generic comparator or cheat.
    // Let's use a simple vector or string key.
    std::mutex map_mutex;
    std::map<std::vector<uint8_t>, std::string> uuid_map;

    // NEW: UUID to scenario_id mapping (for status callback context)
    std::map<std::vector<uint8_t>, std::string> uuid_scenario_map;

    // NEW: Goal handle storage (for cancel operations)
    // goal_id (string) -> goal_handle (shared_ptr)
    std::map<std::string, rclcpp_action::ClientGoalHandle<FibAction>::SharedPtr> goal_handles;

    // NEW: Thread-safe buffer for pending status events
    // Status subscription appends here; ops drain and emit via TraceWriter
    std::mutex status_buffer_mutex;
    std::deque<json> pending_status_events;

    // Thread-safe buffer for pending parameter events.
    std::mutex param_event_buffer_mutex;
    std::deque<json> pending_param_events;

    /// goal_id -> terminal_status (e.g. "SUCCEEDED")
    std::unordered_map<std::string, std::string> terminal;
    /// goal_id -> terminal set attempts count
    std::unordered_map<std::string, uint32_t> terminal_attempts;

    /// Actor name -> PID
    std::map<std::string, pid_t> pid_map;

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
