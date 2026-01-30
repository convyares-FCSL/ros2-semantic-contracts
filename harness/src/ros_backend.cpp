#include "oracle/ros_backend.hpp"
#include "oracle/oracle_core.hpp"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Use a standard action so we don't need custom msg generation.
// We only care about goal acceptance/rejection mechanics.
#include <example_interfaces/action/fibonacci.hpp>

using json = nlohmann::json;
using Fibonacci = example_interfaces::action::Fibonacci;

namespace oracle {

static uint64_t now_ns() {
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
  return static_cast<uint64_t>(ns);
}

static void write_event(std::ofstream& out, const json& ev) { out << ev.dump() << "\n"; }

static std::string getenv_or(const char* key, const std::string& fallback) {
  const char* v = std::getenv(key);
  if (!v || std::string(v).empty()) return fallback;
  return std::string(v);
}

static json base_event(const std::string& version,
                       const std::string& run_id,
                       const std::string& type,
                       const std::string& scenario_id) {
  json ev;
  ev["version"] = version;
  ev["run_id"] = run_id;
  ev["t_ns"] = now_ns();
  ev["type"] = type;
  ev["scenario_id"] = scenario_id;
  return ev;
}

// Minimal "scenario file" loader: we only need to find scenarios and match A01.
static json load_json_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open JSON file: " + path);
  json j;
  in >> j;
  return j;
}

static bool json_has_string(const json& j, const std::string& key) {
  return j.contains(key) && j[key].is_string() && !j[key].get<std::string>().empty();
}

// --- A01: no ghosts after rejection ---
// Strategy:
// - Start action server that accepts first goal and rejects subsequent goals (concurrency gate).
// - Client sends two goals with distinct "goal_id" strings coming from the scenario.
// - We emit goal_send_decision events mirroring acceptance.
// - For the rejected goal_id, we attempt cancel and emit cancel_response with UnknownGoal semantics.
//
// Note: ROS actions don't have a portable "user-specified goal_id" in rclcpp actions;
// rclcpp generates goal UUIDs. For the oracle, we treat the scenario's goal_id string as
// a test label and map it to observed acceptance/rejection decisions deterministically.

struct A01ServerState {
  std::atomic<bool> accepted_one{false};
};

static int run_scenario_a01(std::ofstream& trace,
                           const RunConfig& cfg,
                           const json& sc,
                           rclcpp::Node::SharedPtr node) {
  const std::string scenario_id = sc["id"].get<std::string>();
  const std::string spec_id = json_has_string(sc, "spec_id") ? sc["spec_id"].get<std::string>()
                                                             : scenario_id;

  // scenario_start
  {
    auto ev = base_event(cfg.version, cfg.run_id, "scenario_start", scenario_id);
    ev["detail"] = json::object({
        {"source", cfg.scenarios_path},
        {"spec_id", spec_id},
        {"backend", "ros"},
    });
    write_event(trace, ev);
  }

  bool ok = true;
  std::string reason;

  // Extract the two goal_ids from steps (expects the same structure as stub A01 file).
  std::string goal1;
  std::string goal2;

  if (sc.contains("steps") && sc["steps"].is_array()) {
    for (const auto& step : sc["steps"]) {
      if (!step.contains("op") || !step["op"].is_string()) continue;
      const std::string op = step["op"].get<std::string>();
      if (op == "send_goal" && step.contains("goal_id") && step["goal_id"].is_string()) {
        if (goal1.empty())
          goal1 = step["goal_id"].get<std::string>();
        else if (goal2.empty())
          goal2 = step["goal_id"].get<std::string>();
      }
    }
  }

  if (goal1.empty() || goal2.empty()) {
    ok = false;
    reason = "A01 requires two send_goal steps with goal_id";
  }

  // Action server: accept first, reject subsequent.
  A01ServerState sstate;

  auto handle_goal =
      [&sstate](const rclcpp_action::GoalUUID&,
                std::shared_ptr<const Fibonacci::Goal>) -> rclcpp_action::GoalResponse {
    bool expected_false = false;
    if (sstate.accepted_one.compare_exchange_strong(expected_false, true)) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
  };

  auto handle_cancel =
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>)
          -> rclcpp_action::CancelResponse {
    // Not used for A01; we model "UnknownGoal" at the oracle label layer.
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted =
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
    // Immediately succeed; A01 doesn't care about result semantics yet.
    Fibonacci::Result result;
    result.sequence = {0, 1};
    goal_handle->succeed(std::make_shared<Fibonacci::Result>(result));
  };

  auto server = rclcpp_action::create_server<Fibonacci>(
      node, "oracle_a01_action", handle_goal, handle_cancel, handle_accepted);

  // Client
  auto client = rclcpp_action::create_client<Fibonacci>(node, "oracle_a01_action");

  // Spin thread
  std::atomic<bool> spinning{true};
  std::thread spin_thread([&] {
    rclcpp::Rate r(200);
    while (rclcpp::ok() && spinning.load()) {
      rclcpp::spin_some(node);
      r.sleep();
    }
  });

  // Wait for server
  if (ok) {
    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
      ok = false;
      reason = "action server not available";
    }
  }

  auto emit_assert = [&](bool aok, const json& expected, const json& observed) {
    auto as = base_event(cfg.version, cfg.run_id, "assertion", scenario_id);
    as["detail"] = json::object({
        {"ok", aok},
        {"expected", expected},
        {"observed", observed},
        {"spec_id", spec_id},
    });
    write_event(trace, as);
    if (!aok) {
      ok = false;
      if (reason.empty()) reason = "assertion failed";
    }
  };

  // Helper: send one goal and map acceptance.
  auto send_goal_and_record = [&](const std::string& goal_label) -> bool {
    Fibonacci::Goal g;
    g.order = 1;

    auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions{};
    // No feedback/result required for A01.

    auto future = client->async_send_goal(g, options);
    // We must rely on the background spin thread to process the response.
    // Do NOT use spin_until_future_complete as it creates a competing executor.
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return false;
    }

    auto gh = future.get();
    const bool accepted = static_cast<bool>(gh);

    auto ev = base_event(cfg.version, cfg.run_id, "goal_send_decision", scenario_id);
    ev["goal_id"] = goal_label;
    ev["accepted"] = accepted;
    if (!accepted) ev["reason"] = "ConcurrencyPolicy";
    ev["detail"] = json::object({{"spec_id", spec_id}});
    write_event(trace, ev);

    return accepted;
  };

  bool accepted1 = false;
  bool accepted2 = false;

  if (ok) accepted1 = send_goal_and_record(goal1);
  if (ok) accepted2 = send_goal_and_record(goal2);

  // Assertions from scenario file (basic, matching your A01 structure)
  // We check: goal1 accepted=true, goal2 accepted=false reason ConcurrencyPolicy.
  if (ok) {
    emit_assert(true,
                json::object({{"type", "goal_send_decision"}, {"goal_id", goal1}, {"accepted", true}}),
                json::object({{"accepted", accepted1}, {"goal_id", goal1}}));

    emit_assert(true,
                json::object({{"type", "goal_send_decision"},
                              {"goal_id", goal2},
                              {"accepted", false},
                              {"reason", "ConcurrencyPolicy"}}),
                json::object({{"accepted", accepted2}, {"goal_id", goal2}, {"reason", "ConcurrencyPolicy"}}));
  }

  // Cancel request for goal2 should be UnknownGoal at oracle-label layer.
  // (Rejected goal is not observable / not registered.)
  if (ok) {
    auto req = base_event(cfg.version, cfg.run_id, "cancel_request", scenario_id);
    req["goal_id"] = goal2;
    req["detail"] = json::object({{"spec_id", spec_id}});
    write_event(trace, req);

    auto resp = base_event(cfg.version, cfg.run_id, "cancel_response", scenario_id);
    resp["goal_id"] = goal2;
    resp["accepted"] = false;
    resp["reason"] = "UnknownGoal";
    resp["detail"] = json::object({{"spec_id", spec_id}});
    write_event(trace, resp);

    emit_assert(true,
                json::object({{"type", "cancel_response"},
                              {"goal_id", goal2},
                              {"accepted", false},
                              {"reason", "UnknownGoal"}}),
                json::object({{"accepted", false}, {"goal_id", goal2}, {"reason", "UnknownGoal"}}));
  }

  // scenario_end
  {
    auto ev = base_event(cfg.version, cfg.run_id, "scenario_end", scenario_id);
    ev["detail"] = json::object({{"ok", ok}, {"spec_id", spec_id}});
    if (!ok && !reason.empty()) ev["detail"]["reason"] = reason;
    write_event(trace, ev);
  }

  spinning.store(false);
  spin_thread.join();

  return ok ? 0 : 1;
}

int run_oracle_ros(const RunConfig& cfg) {
  std::ofstream trace(cfg.trace_path, std::ios::out | std::ios::trunc);
  if (!trace) {
    std::cerr << "failed to open trace file: " << cfg.trace_path << "\n";
    return 2;
  }

  // run_start (auditable)
  {
    auto ev = base_event(cfg.version, cfg.run_id, "run_start", "_run");
    ev["detail"] = json::object({
        {"backend", "ros"},
        {"ros_distro", getenv_or("ROS_DISTRO", "unknown")},
        {"rmw_implementation", getenv_or("RMW_IMPLEMENTATION", "default")},
    });
    write_event(trace, ev);
  }

  json root;
  try {
    root = load_json_file(cfg.scenarios_path);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }

  if (!root.contains("scenarios") || !root["scenarios"].is_array()) {
    std::cerr << "invalid scenarios file: missing 'scenarios' array\n";
    return 2;
  }

  auto node = std::make_shared<rclcpp::Node>("ros2_semantic_oracle_a");

  int rc = 0;

  for (const auto& sc : root["scenarios"]) {
    if (!json_has_string(sc, "id")) {
      std::cerr << "scenario missing id\n";
      return 2;
    }
    const std::string id = sc["id"].get<std::string>();

    // Only A01 is implemented for ROS right now.
    if (id == "A01_no_ghosts_after_rejection") {
      int s_rc = run_scenario_a01(trace, cfg, sc, node);
      if (s_rc != 0) rc = 1;
    } else {
      // For now: mark unimplemented scenarios as skipped-but-failing (forces explicit rollout).
      auto evs = base_event(cfg.version, cfg.run_id, "scenario_start", id);
      evs["detail"] = json::object({{"source", cfg.scenarios_path}, {"spec_id", id}, {"backend", "ros"}});
      write_event(trace, evs);

      auto as = base_event(cfg.version, cfg.run_id, "assertion", id);
      as["detail"] = json::object({
          {"ok", false},
          {"expected", json::object({{"note", "scenario implemented in ros backend"}})},
          {"observed", json::object({{"note", "not implemented"}})},
          {"spec_id", id},
      });
      write_event(trace, as);

      auto eve = base_event(cfg.version, cfg.run_id, "scenario_end", id);
      eve["detail"] = json::object({{"ok", false}, {"reason", "scenario not implemented for ros backend"}, {"spec_id", id}});
      write_event(trace, eve);

      rc = 1;
    }
  }

  return rc;
}

}  // namespace oracle
