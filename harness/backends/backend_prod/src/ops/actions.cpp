/// @file actions.cpp
/// @brief Action-specific operations (send_goal, etc.).
#include "../backend.hpp"
#include <stdexcept>

namespace backend_prod::ops {

/// Executes send_goal: emit goal_send + goal_response.
/// Decision driven by payload.mode (accept|reject).
void exec_send_goal(TraceWriter& w, const std::string& scenario_id, const Op& op) {
    if (!op.goal_id) {
        throw std::runtime_error("send_goal: missing goal_id");
    }
    const std::string& goal_id = *op.goal_id;

    if (!op.payload || !op.payload->contains("mode")) {
        throw std::runtime_error("send_goal: missing payload.mode");
    }
    std::string mode = (*op.payload)["mode"].get<std::string>();

    w.emit({{"type", "goal_send"}, {"scenario_id", scenario_id}, {"goal_id", goal_id}});

    bool accepted = (mode == "accept");
    json response = {
        {"type", "goal_response"},
        {"scenario_id", scenario_id},
        {"goal_id", goal_id},
        {"accepted", accepted}
    };
    if (!accepted) {
        response["reason"] = "prod_reject";
    }
    w.emit(response);
}

} // namespace backend_prod::ops
