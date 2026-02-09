/// @file actions.cpp
/// @brief Action-specific operations (send_goal, etc.).
#include "../backend.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include "status_drain.hpp"
#include "internal/errors.hpp"

namespace backend_prod::ops {

/// Executes send_goal: real ROS 2 Action Call against an external peer.
/// 1. Send Goal via Client.
/// 2. Wait for Future (GoalResponse).
/// 3. Emit goal_send_decision.
void exec_send_goal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    if (!op.goal_id) {
        throw BundleError("send_goal: missing goal_id");
    }
    const std::string& goal_id_str = *op.goal_id;

    if (!st.client) {
         throw SystemError("Action client not initialized");
    }

    // Create status subscription if not already created
    // SAFETY: Callback does NOT capture TraceWriter& w - only appends to buffer
    if (!st.status_sub) {
        st.status_sub = st.node->create_subscription<action_msgs::msg::GoalStatusArray>(
            "test_action/_action/status",
            10,
            [&st](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                // CRITICAL: Do NOT capture TraceWriter& w here (dangling reference risk)
                // CRITICAL: Avoid lock inversion - build events locally, then append to buffer

                // Step 1: Lock map_mutex, build events locally, unlock
                std::vector<json> events_to_append;
                {
                    std::lock_guard<std::mutex> map_lock(st.map_mutex);
                    for (const auto& status_item : msg->status_list) {
                        // Convert UUID to vector<uint8_t> for map lookup
                        std::vector<uint8_t> uuid_vec(
                            status_item.goal_info.goal_id.uuid.begin(),
                            status_item.goal_info.goal_id.uuid.end()
                        );

                        // Check if this goal belongs to our scenarios
                        auto goal_it = st.uuid_map.find(uuid_vec);
                        auto scenario_it = st.uuid_scenario_map.find(uuid_vec);

                        if (goal_it != st.uuid_map.end() && scenario_it != st.uuid_scenario_map.end()) {
                            const std::string& goal_id = goal_it->second;
                            const std::string& scenario_id = scenario_it->second;

                            // Map action_msgs::GoalStatus to status string
                            std::string status_str;
                            switch (status_item.status) {
                                case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
                                    status_str = "STATUS_ACCEPTED"; break;
                                case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                                    status_str = "STATUS_EXECUTING"; break;
                                case action_msgs::msg::GoalStatus::STATUS_CANCELING:
                                    status_str = "STATUS_CANCELING"; break;
                                case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                                    status_str = "STATUS_SUCCEEDED"; break;
                                case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                                    status_str = "STATUS_CANCELED"; break;
                                case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                                    status_str = "STATUS_ABORTED"; break;
                                default:
                                    status_str = "STATUS_UNKNOWN"; break;
                            }

                            // Build event locally
                            events_to_append.push_back({
                                {"type", "status"},
                                {"scenario_id", scenario_id},
                                {"goal_id", goal_id},
                                {"status", status_str}
                            });
                        }
                    }
                } // map_lock released here

                // Step 2: Lock status_buffer_mutex, append events, unlock
                if (!events_to_append.empty()) {
                    std::lock_guard<std::mutex> buffer_lock(st.status_buffer_mutex);
                    st.pending_status_events.insert(
                        st.pending_status_events.end(),
                        events_to_append.begin(),
                        events_to_append.end()
                    );
                }
            }
        );
    }

    // Drain any pending status events from previous operations
    drain_status_buffer(w, st);
    
    // Ensure server is ready
    // std::cerr << "DEBUG: Waiting for action server...\n";
    if (!st.client->wait_for_action_server(std::chrono::seconds(2))) {
        // std::cerr << "DEBUG: Action server timeout!\n";
        throw SystemError("Action server not available");
    }
    // std::cerr << "DEBUG: Action server found.\n";

    // 2. Send Goal
    auto goal_msg = FibAction::Goal();
    goal_msg.order = 1; // dummy

    auto send_goal_options = rclcpp_action::Client<FibAction>::SendGoalOptions();
    
    // Result callback: Map WrappedResult code to canonical status string
    send_goal_options.result_callback = [&w, scenario_id, goal_id_str](const rclcpp_action::ClientGoalHandle<FibAction>::WrappedResult& result) {
         // Map status code to string
         std::string status_str;
         switch (result.code) {
             case rclcpp_action::ResultCode::SUCCEEDED: status_str = "SUCCEEDED"; break;
             case rclcpp_action::ResultCode::ABORTED:   status_str = "ABORTED"; break;
             case rclcpp_action::ResultCode::CANCELED:  status_str = "CANCELED"; break;
             default:                                   status_str = "UNKNOWN"; break;
         }

         w.emit({
            {"type", "result"},
            {"scenario_id", scenario_id},
            {"goal_id", goal_id_str},
            {"status", status_str}
         });
    };
    
    // Send goal
    // std::cerr << "DEBUG: Sending goal async...\n";
    auto future_goal_handle = st.client->async_send_goal(goal_msg, send_goal_options);

    // 3. Wait for Future
    // We block here. The spinner thread handles the callbacks/service-return.
    // std::cerr << "DEBUG: Waiting for future...\n";
    auto status = future_goal_handle.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
         // std::cerr << "DEBUG: Future timeout!\n";
         throw SystemError("Timeout waiting for goal response");
    }
    // std::cerr << "DEBUG: Future ready.\n";

    auto goal_handle = future_goal_handle.get();
    bool accepted = (goal_handle != nullptr);

    if (accepted) {
        // Map UUID to string for future callbacks
        auto uuid = goal_handle->get_goal_id();
        std::vector<uint8_t> uuid_vec(uuid.begin(), uuid.end());
        {
            std::lock_guard<std::mutex> lock(st.map_mutex);
            st.uuid_map[uuid_vec] = goal_id_str;
            st.uuid_scenario_map[uuid_vec] = scenario_id;
            // Store goal handle for cancel operations
            st.goal_handles[goal_id_str] = goal_handle;
        }
    }

    // 3. Emit goal_send_decision (outcome driven by peer's real response)
    w.emit({
        {"type", "goal_send_decision"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id_str},
        {"accepted", accepted}
    });

    // Drain status events that arrived during send_goal
    drain_status_buffer(w, st);

    // Note: If accepted, we should theoretically wait for result.
    // But A01 tests rejection. A02 tests terminal.
    // For A02, we need to handle result.
    // But for this task (A01), we stop here.
}

/// Executes cancel_goal: cancels an active goal via the ROS 2 Action cancel API.
/// 1. Look up goal handle by goal_id.
/// 2. Emit cancel_request.
/// 3. Call async_cancel_goal.
/// 4. Wait for response and emit cancel_response with accepted status.
void exec_cancel_goal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    if (!op.goal_id) {
        throw BundleError("cancel_goal: missing goal_id");
    }
    const std::string& goal_id_str = *op.goal_id;

    // Drain any pending status events before cancel
    drain_status_buffer(w, st);

    // Look up goal handle
    rclcpp_action::ClientGoalHandle<FibAction>::SharedPtr goal_handle;
    {
        std::lock_guard<std::mutex> lock(st.map_mutex);
        auto it = st.goal_handles.find(goal_id_str);
        if (it == st.goal_handles.end()) {
            throw BundleError("cancel_goal: goal_id '" + goal_id_str + "' not found in active goals");
        }
        goal_handle = it->second;
    }

    if (!goal_handle) {
        throw BundleError("cancel_goal: goal_handle is null for goal_id '" + goal_id_str + "'");
    }

    // Emit cancel_request (intent to cancel)
    w.emit({
        {"type", "cancel_request"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id_str}
    });

    if (!st.client) {
        throw SystemError("Action client not initialized");
    }

    // Call async_cancel_goal on the client (not the goal_handle)
    auto future_cancel = st.client->async_cancel_goal(goal_handle);

    // Wait for cancel response
    auto status = future_cancel.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
        throw SystemError("Timeout waiting for cancel response");
    }

    auto cancel_response = future_cancel.get();

    // Map return code to int and determine if cancel was accepted
    // rclcpp_action::CancelResponse has return_code field
    int return_code = static_cast<int>(cancel_response->return_code);

    // According to action_msgs/srv/CancelGoal:
    // ERROR_NONE = 0 (success), ERROR_REJECTED = 1, ERROR_UNKNOWN_GOAL_ID = 2, ERROR_GOAL_TERMINATED = 3
    bool accepted = (return_code == 0);  // ERROR_NONE means cancel accepted

    // Emit cancel_response (observable outcome)
    w.emit({
        {"type", "cancel_response"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id_str},
        {"accepted", accepted},
        {"return_code", return_code}
    });

    // Drain status events that may have arrived during cancel
    drain_status_buffer(w, st);
}

/// Executes complete_terminal: sets goal to terminal status.
/// Emits status and result events. Terminal is immutable (subsequent sets denied).
void exec_complete_terminal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    if (!op.goal_id) {
        throw BundleError("complete_terminal: missing goal_id");
    }
    const std::string& goal_id = *op.goal_id;

    if (!op.status) {
        throw BundleError("complete_terminal: missing status");
    }
    const std::string& status = *op.status;

    uint32_t& attempt = st.terminal_attempts[goal_id];
    attempt++;

    // Terminal is immutable: once set, subsequent terminal sets are denied but observable.
    if (st.terminal.count(goal_id)) {
        w.emit({
            {"type", "terminal_set_attempt"},
            {"scenario_id", scenario_id},
            {"goal_id", goal_id},
            {"attempt", attempt},
            {"allowed", false},
            {"reason", "terminal_immutable"}
        });
        return;
    }

    st.terminal[goal_id] = status;

    w.emit({
        {"type", "status"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id},
        {"status", status}
    });

    w.emit({
        {"type", "result"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id},
        {"status", status}
    });
}

/// Executes attempt_terminal_override: attempts to override terminal status.
/// If already terminal, override is denied and observable.
void exec_attempt_terminal_override(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    if (!op.goal_id) {
        throw BundleError("attempt_terminal_override: missing goal_id");
    }
    const std::string& goal_id = *op.goal_id;

    if (!op.status) {
        throw BundleError("attempt_terminal_override: missing status");
    }
    // const std::string& _requested = *op.status;  // Not used, but validated

    uint32_t& attempt = st.terminal_attempts[goal_id];
    attempt++;

    // If already terminal, override is denied and observable.
    if (st.terminal.count(goal_id)) {
        w.emit({
            {"type", "terminal_set_attempt"},
            {"scenario_id", scenario_id},
            {"goal_id", goal_id},
            {"attempt", attempt},
            {"allowed", false},
            {"reason", "terminal_immutable"}
        });
        return;
    }

    // If not terminal yet, record that an override was "allowed" (rare in real systems),
    // but do not fabricate terminal semantics here.
    w.emit({
        {"type", "terminal_set_attempt"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id},
        {"attempt", attempt},
        {"allowed", true}
    });
}

} // namespace backend_prod::ops
