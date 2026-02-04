/// @file actions.cpp
/// @brief Action-specific operations (send_goal, etc.).
#include "../backend.hpp"
#include "internal/errors.hpp"

namespace backend_prod::ops {

/// Executes send_goal: real ROS 2 Action Call.
/// 1. Configure "Rig" (Server) policy.
/// 2. Send Goal via Client.
/// 3. Wait for Future (GoalResponse).
/// 4. Emit goal_send_decision.
void exec_send_goal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    if (!op.goal_id) {
        throw BundleError("send_goal: missing goal_id");
    }
    const std::string& goal_id_str = *op.goal_id;

    if (!op.payload || !op.payload->contains("mode")) {
        throw BundleError("send_goal: missing payload.mode");
    }
    std::string mode = (*op.payload)["mode"].get<std::string>();

    // 1. Configure Policy
    if (mode == "reject") {
        st.next_goal_policy.store(1);
    } else {
        st.next_goal_policy.store(0);
    }

    if (!st.client) {
         throw SystemError("Action client not initialized");
    }
    
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
    
    // Callbacks to detect "Ghosts" (unexpected feedback/result for rejected goals)
    send_goal_options.result_callback = [&w, scenario_id, goal_id_str](const rclcpp_action::ClientGoalHandle<FibAction>::WrappedResult& result) {
         // std::cerr << "DEBUG: Result Callback hit for " << goal_id_str << "\n";
         w.emit({
            {"type", "result"},
            {"scenario_id", scenario_id},
            {"goal_id", goal_id_str},
            {"status", "UNKNOWN_RESULT_CALLBACK"} // Placeholder, purely to fail "Absent" checks
         });
    };

    // We can't force the UUID easily in rclcpp_action without hacking. 
    // We Rely on mapping. 
    // Problem: We need the UUID *before* we send? No, we get it from the handle.
    // But handle comes *after* response?
    // Actually, future resolves to handle. Handle has ID.
    
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
        }
    }

    // 4. Emit goal_send_decision
    json ev = {
        {"type", "goal_send_decision"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id_str},
        {"accepted", accepted}
    };
    if (!accepted) {
        ev["reason"] = "policy_reject";
    }
    w.emit(ev);

    // Note: If accepted, we should theoretically wait for result. 
    // But A01 tests rejection. A02 tests terminal.
    // For A02, we need to handle result.
    // But for this task (A01), we stop here.
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
