/// @file backend.cpp
/// @brief Scenario execution orchestration.
#include "backend_prod/backend.hpp"
#include "backend.hpp"
#include "internal/bundle_parser.hpp"
#include "internal/fs_utils.hpp"
#include <cstdlib>

namespace backend_prod {

// Forward declarations from ops/
namespace ops {
void exec_send_goal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
void exec_wait(const Op& op);
json exec_get_clock(TraceWriter& w, const std::string& scenario_id);
void exec_complete_terminal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
void exec_attempt_terminal_override(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
}

void exec_op(TraceWriter& w, BackendState& st, const std::string& scenario_id, const std::string& op_id, const Op& op) {
    w.emit({
        {"type", "op_start"},
        {"scenario_id", scenario_id},
        {"op_id", op_id},
        {"detail", {{"op", op.op}}}
    });

    json op_result;  // Optional result to include in op_end.detail

    if (op.op == "send_goal") {
        ops::exec_send_goal(w, st, scenario_id, op);
    } else if (op.op == "wait") {
        ops::exec_wait(op);
    } else if (op.op == "get_clock") {
        op_result = ops::exec_get_clock(w, scenario_id);
    } else if (op.op == "complete_terminal") {
        ops::exec_complete_terminal(w, st, scenario_id, op);
    } else if (op.op == "attempt_terminal_override") {
        ops::exec_attempt_terminal_override(w, st, scenario_id, op);
    } else {
        throw BundleError("unsupported op: " + op.op);
    }

    // Emit op_end with optional result in detail
    json op_end = {{"type", "op_end"}, {"scenario_id", scenario_id}, {"op_id", op_id}};
    if (!op_result.is_null()) {
        op_end["detail"] = op_result;
    }
    w.emit(op_end);
}

void run_backend(const std::string& bundle_path, const std::string& trace_path) {
    // Parse and validate bundle
    Bundle bundle = parse_bundle(bundle_path);
    
    // Ensure trace directory exists
    ensure_parent_dir(trace_path);
    
    // Configure run_id from environment
    const char* env_run_id = std::getenv("BACKEND_PROD_RUN_ID");
    std::string run_id = env_run_id ? env_run_id : "run_prod";
    
    // Open trace writer
    TraceWriter w(trace_path, run_id);

    // run_start
    w.emit({{"type", "run_start"}, {"scenario_id", "_run"}, {"detail", {{"backend", "prod"}}}});

    // backend_capabilities
    w.emit({
        {"type", "backend_capabilities"},
        {"scenario_id", "_run"},
        {"detail", {
            {"backend", "prod"},
            {"caps", json::array({"actions.basic", "actions.terminal", "clock.ros_time"})},
            {"limits", json::object()}
        }}
    });

    // Execute scenarios in sorted order (std::map is sorted by key).
    for (const auto& [id, scenario] : bundle.scenarios) {
        w.emit({{"type", "scenario_start"}, {"scenario_id", id}, {"detail", {{"backend", "prod"}}}});

        // Create per-scenario state with ROS infrastructure
        BackendState st;
        st.node = std::make_shared<rclcpp::Node>("backend_prod_client_" + id);
        st.server_node = std::make_shared<rclcpp::Node>("backend_prod_server_" + id);
        
        st.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        st.executor->add_node(st.node);
        st.executor->add_node(st.server_node);

        // Action Client
        st.client = rclcpp_action::create_client<FibAction>(st.node, "test_action");

        // Action Server (Rig)
        st.server = rclcpp_action::create_server<FibAction>(
            st.server_node,
            "test_action",
            [&st](const rclcpp_action::GoalUUID&, std::shared_ptr<const FibAction::Goal>) {
                // Goal Callback: Check policy
                int policy = st.next_goal_policy.load();
                // std::cerr << "RIG: Goal Request received. Policy=" << policy << "\n";
                if (policy == 1) {
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>&) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>&) {
                // Execute
                std::thread{[](){}}.detach(); 
            }
        );

        // Start spinner
        st.spinner_thread = std::thread([&st]() {
            st.executor->spin();
        });

        int idx = 0;
        for (const auto& op : scenario.ops) {
            std::string op_id = id + "#" + std::to_string(idx++);
            exec_op(w, st, id, op_id, op);
        }

        w.emit({{"type", "scenario_end"}, {"scenario_id", id}});
    }

    // run_end
    w.emit({{"type", "run_end"}, {"scenario_id", "_run"}});
}

} // namespace backend_prod
