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
void exec_send_goal(TraceWriter& w, const std::string& scenario_id, const Op& op);
void exec_wait(const Op& op);
}

void exec_op(TraceWriter& w, const std::string& scenario_id, const std::string& op_id, const Op& op) {
    w.emit({
        {"type", "op_start"},
        {"scenario_id", scenario_id},
        {"op_id", op_id},
        {"detail", {{"op", op.op}}}
    });

    if (op.op == "send_goal") {
        ops::exec_send_goal(w, scenario_id, op);
    } else if (op.op == "wait") {
        ops::exec_wait(op);
    } else {
        throw BundleError("unsupported op: " + op.op);
    }

    w.emit({{"type", "op_end"}, {"scenario_id", scenario_id}, {"op_id", op_id}});
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
            {"caps", json::array({"actions.basic"})},
            {"limits", json::object()}
        }}
    });

    // Execute scenarios in sorted order (std::map is sorted by key).
    for (const auto& [id, scenario] : bundle.scenarios) {
        w.emit({{"type", "scenario_start"}, {"scenario_id", id}, {"detail", {{"backend", "prod"}}}});

        int idx = 0;
        for (const auto& op : scenario.ops) {
            std::string op_id = id + "#" + std::to_string(idx++);
            exec_op(w, id, op_id, op);
        }

        w.emit({{"type", "scenario_end"}, {"scenario_id", id}});
    }

    // run_end
    w.emit({{"type", "run_end"}, {"scenario_id", "_run"}});
}

} // namespace backend_prod
