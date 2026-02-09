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
void exec_cancel_goal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
void exec_wait(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_get_clock(TraceWriter& w, const std::string& scenario_id);
void exec_complete_terminal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
void exec_attempt_terminal_override(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_declare_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_set_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_get_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_describe_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_set_params_batch(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_set_params_non_atomic(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_register_param_callback(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_register_pre_set_callback(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_observe_param_events(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_list_params(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_get_parameter_types(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_concurrent_read_during_batch(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_undeclare_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
}

namespace runtime {
json exec_start_actor(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
json exec_stop_actor(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
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
    } else if (op.op == "cancel_goal") {
        ops::exec_cancel_goal(w, st, scenario_id, op);
    } else if (op.op == "wait") {
        ops::exec_wait(w, st, scenario_id, op);
    } else if (op.op == "get_clock") {
        op_result = ops::exec_get_clock(w, scenario_id);
    } else if (op.op == "complete_terminal") {
        ops::exec_complete_terminal(w, st, scenario_id, op);
    } else if (op.op == "attempt_terminal_override") {
        ops::exec_attempt_terminal_override(w, st, scenario_id, op);
    } else if (op.op == "start_actor") {
        op_result = runtime::exec_start_actor(w, st, scenario_id, op);
    } else if (op.op == "stop_actor") {
        op_result = runtime::exec_stop_actor(w, st, scenario_id, op);
    } else if (op.op == "declare_param") {
        op_result = ops::exec_declare_param(w, st, scenario_id, op);
    } else if (op.op == "set_param") {
        op_result = ops::exec_set_param(w, st, scenario_id, op);
    } else if (op.op == "get_param") {
        op_result = ops::exec_get_param(w, st, scenario_id, op);
    } else if (op.op == "describe_param") {
        op_result = ops::exec_describe_param(w, st, scenario_id, op);
    } else if (op.op == "set_params_batch") {
        op_result = ops::exec_set_params_batch(w, st, scenario_id, op);
    } else if (op.op == "set_params_non_atomic") {
        op_result = ops::exec_set_params_non_atomic(w, st, scenario_id, op);
    } else if (op.op == "register_param_callback") {
        op_result = ops::exec_register_param_callback(w, st, scenario_id, op);
    } else if (op.op == "register_pre_set_callback") {
        op_result = ops::exec_register_pre_set_callback(w, st, scenario_id, op);
    } else if (op.op == "observe_param_events") {
        op_result = ops::exec_observe_param_events(w, st, scenario_id, op);
    } else if (op.op == "list_params") {
        op_result = ops::exec_list_params(w, st, scenario_id, op);
    } else if (op.op == "get_parameter_types") {
        op_result = ops::exec_get_parameter_types(w, st, scenario_id, op);
    } else if (op.op == "concurrent_read_during_batch") {
        op_result = ops::exec_concurrent_read_during_batch(w, st, scenario_id, op);
    } else if (op.op == "undeclare_param") {
        op_result = ops::exec_undeclare_param(w, st, scenario_id, op);
    } else {
        throw BundleError("unsupported op: '" + op.op + "'");
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
            {"caps", json::array({
                "actions.basic",
                "actions.terminal",
                "clock.ros_time",
                "ros.params.declare",
                "ros.params.set",
                "ros.params.get",
                "ros.params.set_batch",
                "ros.params.set_non_atomic",
                "ros.params.describe",
                "ros.params.delete",
                "ros.params.list",
                "ros.params.callback",
                "ros.params.pre_set_callback",
                "ros.params.observe_events",
                "ros.params.get_parameter_types",
                "ros.params.concurrent_read"
            })},
            {"limits", json::object()}
        }}
    });

    // Execute scenarios in sorted order (std::map is sorted by key).
    for (const auto& [id, scenario] : bundle.scenarios) {
        w.emit({{"type", "scenario_start"}, {"scenario_id", id}, {"detail", {{"backend", "prod"}}}});

        if (scenario.ops.empty()) {
            // No ops — nothing to execute.  Emit end and move on without
            // spinning up ROS infrastructure.
            w.emit({{"type", "scenario_end"}, {"scenario_id", id}});
            continue;
        }

        // Create per-scenario state with ROS infrastructure (client-only)
        BackendState st;
        bool needs_action_spinner = false;
        for (const auto& scenario_op : scenario.ops) {
            if (
                scenario_op.op == "send_goal" ||
                scenario_op.op == "cancel_goal" ||
                scenario_op.op == "complete_terminal" ||
                scenario_op.op == "attempt_terminal_override"
            ) {
                needs_action_spinner = true;
                break;
            }
        }

        auto opts = rclcpp::NodeOptions()
            .allow_undeclared_parameters(false)
            .automatically_declare_parameters_from_overrides(false);
        st.node = std::make_shared<rclcpp::Node>("backend_prod_client_" + id, opts);

        st.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        st.executor->add_node(st.node);

        if (needs_action_spinner) {
            // Action Client (peer server must already be running)
            st.client = rclcpp_action::create_client<FibAction>(st.node, "test_action");
            // Action flows require background spinning for callbacks/futures.
            st.spinner_thread = std::thread([&st]() {
                st.executor->spin();
            });
        }

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
