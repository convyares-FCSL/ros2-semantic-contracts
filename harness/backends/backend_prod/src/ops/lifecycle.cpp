/// @file lifecycle.cpp
/// @brief Lifecycle operations for backend_prod (matching rclpy/L05/S11 requirements)
#include "../backend.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>

namespace backend_prod::ops {

using json = nlohmann::json;

// --- SlowLifecycleNode Implementation ---

class SlowLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit SlowLifecycleNode(const std::string& node_name)
        : rclcpp_lifecycle::LifecycleNode(node_name), transition_delay_ms_(0), in_transition_(false) {}

    // Configuration for delays (S11/L05)
    void set_transition_delay(int ms) {
        transition_delay_ms_ = ms;
    }

    // Manual lock for concurrency test (L05)
    bool try_lock_transition() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (in_transition_) return false;
        in_transition_ = true;
        return true;
    }

    void unlock_transition() {
        std::lock_guard<std::mutex> lock(mutex_);
        in_transition_ = false;
    }

    // Override callbacks to inject delay
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override {
        sleep_if_needed();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override {
        sleep_if_needed();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override {
        sleep_if_needed();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override {
        sleep_if_needed();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void sleep_if_needed() {
        if (transition_delay_ms_ > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(transition_delay_ms_));
        }
    }

    int transition_delay_ms_;
    bool in_transition_;
    std::mutex mutex_;
};

// --- Operations ---

json exec_init_lifecycle_node(NodeContext& ctx, const Op& op, const std::string& scenario_id) {
    // Instantiate SlowLifecycleNode
    auto lifecycle_node = std::make_shared<SlowLifecycleNode>("probe_node_" + scenario_id);
    ctx.lifecycle_node = lifecycle_node;
    // We do NOT clear ctx.node, we just prioritize lifecycle_node in ops that check.
    // However, P04 expects ctx.node. L05 uses this init, so subsequently we should use lifecycle_node.
    return json::object();
}

json exec_change_state(NodeContext& ctx, const Op& op) {
    if (!ctx.lifecycle_node) return {{"successful", false}, {"error", "Lifecycle node not initialized"}};
    
    // We cast to our specific class to access test hooks
    auto ln = std::dynamic_pointer_cast<SlowLifecycleNode>(ctx.lifecycle_node);
    if (!ln) return {{"successful", false}, {"error", "Node is not a SlowLifecycleNode"}};

    std::string goal = (*op.payload)["goal_state"];
    ln->set_transition_delay(0);

    // Check concurrency (L05)
    if (!ln->try_lock_transition()) {
         return {{"goal", goal}, {"successful", false}, {"code", "BUSY"}};
    }

    bool success = false;
    try {
        if (goal == "configured") success = (ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        else if (goal == "inactive") success = (ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP).id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
        else if (goal == "active") success = (ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        else if (goal == "finalized") success = (ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN).id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED); 
        else if (goal == "unconfigured") success = (ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP).id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
        else success = false;

    } catch (...) {
        success = false;
    }
    
    ln->unlock_transition();
    
    return {{"goal", goal}, {"successful", success}};
}

json exec_change_state_async(NodeContext& ctx, const Op& op) {
    if (!ctx.lifecycle_node) return {{"successful", false}, {"error", "Lifecycle node not initialized"}};
    auto ln = std::dynamic_pointer_cast<SlowLifecycleNode>(ctx.lifecycle_node);
        
    std::string goal = (*op.payload)["goal_state"];
    int delay_ms = (*op.payload).value("delay_ms", 0);
    
    ln->set_transition_delay(delay_ms);
    
    // Lock immediately
    if (!ln->try_lock_transition()) {
         return {{"goal", goal}, {"successful", false}, {"code", "BUSY"}};
    }

    // Spawn thread
    std::thread([ln, goal]() {
        try {
            if (goal == "configured") ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            else if (goal == "active") ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            else if (goal == "inactive") ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
            else if (goal == "finalized") ln->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
        } catch (...) {}
        ln->unlock_transition();
    }).detach();

    return {{"goal", goal}, {"status", "async_started"}};
}

json exec_get_state(NodeContext& ctx, const Op& op) {
    if (!ctx.lifecycle_node) return {{"error", "Lifecycle node not initialized"}};
    
    auto ln = ctx.lifecycle_node;
    int deadline_ms = (*op.payload).value("deadline_ms", 1000);
    
    auto start = std::chrono::steady_clock::now();

    auto state = ln->get_current_state();
    std::string label = state.label();
    
    std::transform(label.begin(), label.end(), label.begin(), ::tolower);

    auto end = std::chrono::steady_clock::now();
    int latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    json result = {
        {"state", label},
        {"latency_ms", latency_ms}
    };
    
    if (latency_ms > deadline_ms) {
        result["deadline_exceeded"] = true;
        result["deadline_ms"] = deadline_ms;
    }
    
    return result;
}

} // namespace backend_prod::ops
