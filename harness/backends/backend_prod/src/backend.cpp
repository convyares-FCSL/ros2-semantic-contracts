/// @file backend.cpp
/// @brief Scenario execution orchestration.
#include "backend_prod/backend.hpp"
#include "backend.hpp"
#include "internal/bundle_parser.hpp"
#include "internal/fs_utils.hpp"
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

namespace backend_prod {

// Forward declarations from ops/
namespace ops {
void exec_send_goal(TraceWriter& w, const std::string& scenario_id, const Op& op);
void exec_wait(const Op& op);
// Params
using json = nlohmann::json;
json exec_declare_param(NodeContext& ctx, const Op& op);
json exec_set_params_batch(NodeContext& ctx, const Op& op);
// Lifecycle
json exec_init_lifecycle_node(NodeContext& ctx, const Op& op, const std::string& scenario_id);
json exec_change_state(NodeContext& ctx, const Op& op);
json exec_change_state_async(NodeContext& ctx, const Op& op);
json exec_get_state(NodeContext& ctx, const Op& op);
}

void exec_op(TraceWriter& w, NodeContext& ctx, const std::string& scenario_id, const std::string& op_id, const Op& op) {
    auto start_payload = op.payload ? *op.payload : nlohmann::json::object();
    w.emit({
        {"type", "op_start"},
        {"scenario_id", scenario_id},
        {"op_id", op_id},
        {"detail", {{"op", op.op}, {"payload", start_payload}}}
    });

    nlohmann::json result_detail = nlohmann::json::object();

    if (op.op == "send_goal") {
        ops::exec_send_goal(w, scenario_id, op);
    } else if (op.op == "wait") {
        ops::exec_wait(op);
    } else if (op.op == "declare_param") {
        result_detail = ops::exec_declare_param(ctx, op);
    } else if (op.op == "set_params_batch") {
        result_detail = ops::exec_set_params_batch(ctx, op);
    } else if (op.op == "init_lifecycle_node") {
        result_detail = ops::exec_init_lifecycle_node(ctx, op, scenario_id);
    } else if (op.op == "change_state") {
        result_detail = ops::exec_change_state(ctx, op);
    } else if (op.op == "change_state_async") {
        result_detail = ops::exec_change_state_async(ctx, op);
    } else if (op.op == "get_state") {
        result_detail = ops::exec_get_state(ctx, op);
    } else {
        throw BundleError("unsupported op: " + op.op);
    }

    w.emit({{"type", "op_end"}, {"scenario_id", scenario_id}, {"op_id", op_id}, {"detail", {{"result", result_detail}}}});
}

void run_backend(const std::string& bundle_path, const std::string& trace_path) {
    // Parse and validate bundle
    Bundle bundle = parse_bundle(bundle_path);
    
    // Ensure trace directory exists
    ensure_parent_dir(trace_path);
    
    // Configure run_id from environment
    const char* env_run_id = std::getenv("BACKEND_PROD_RUN_ID");
    std::string run_id = env_run_id ? env_run_id : "run_prod";
    
    // Create Node (default standard node)
    NodeContext ctx;
    ctx.node = std::make_shared<rclcpp::Node>("backend_prod_node");

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
            {"caps", json::array({"actions.basic", "ros.params.declare", "ros.params.set_batch"})},
            {"limits", json::object()}
        }}
    });

    // Execute scenarios in sorted order (std::map is sorted by key).
    for (const auto& [id, scenario] : bundle.scenarios) {
        w.emit({{"type", "scenario_start"}, {"scenario_id", id}, {"detail", {{"backend", "prod"}}}});

        int idx = 0;
        for (const auto& op : scenario.ops) {
            std::string op_id = id + "#" + std::to_string(idx++);
            exec_op(w, ctx, id, op_id, op);
        }

        w.emit({{"type", "scenario_end"}, {"scenario_id", id}});
    }

    // run_end
    w.emit({{"type", "run_end"}, {"scenario_id", "_run"}});
}

} // namespace backend_prod
