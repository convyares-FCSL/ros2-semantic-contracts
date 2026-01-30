#include "oracle/ros_backend.hpp"

#include "oracle/trace_writer.hpp"
#include "oracle/util.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <example_interfaces/action/fibonacci.hpp>

namespace oracle {

using util::json;
using Fibonacci = example_interfaces::action::Fibonacci;

namespace {

class SpinThread {
 public:
  explicit SpinThread(const rclcpp::Node::SharedPtr& node) : node_(node) {}

  void start() {
    running_.store(true);
    th_ = std::thread([this] {
      rclcpp::Rate r(200);
      while (rclcpp::ok() && running_.load()) {
        rclcpp::spin_some(node_);
        r.sleep();
      }
    });
  }

  void stop() {
    running_.store(false);
    if (th_.joinable()) th_.join();
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> running_{false};
  std::thread th_;
};

class ActionRig {
 public:
  virtual ~ActionRig() = default;

  virtual rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID& uuid,
                                              std::shared_ptr<const Fibonacci::Goal> goal) = 0;

  virtual rclcpp_action::CancelResponse on_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>& /*handle*/) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void on_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>& handle) {
    // Default: do nothing. Server handle is captured by interpreter via shared state.
    (void)handle;
  }
};

class ConcurrencyGateRig final : public ActionRig {
 public:
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID&,
                                      std::shared_ptr<const Fibonacci::Goal>) override {
    bool expected = false;
    if (accepted_one_.compare_exchange_strong(expected, true)) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
  }

 private:
  std::atomic<bool> accepted_one_{false};
};

class SimpleRig final : public ActionRig {
 public:
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID&,
                                      std::shared_ptr<const Fibonacci::Goal>) override {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
};

struct RosState {
  // scenario goal label -> client handle
  std::unordered_map<std::string, rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr> client_handles;
  // scenario goal label -> server handle
  std::unordered_map<std::string, std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>> server_handles;
};

class RosScenarioRunner {
 public:
  RosScenarioRunner(std::ofstream& out,
                    const RunConfig& cfg,
                    const json& scenario,
                    const rclcpp::Node::SharedPtr& node,
                    std::shared_ptr<ActionRig> rig)
      : out_(out),
        trace_(out_),
        cfg_(cfg),
        sc_(scenario),
        node_(node),
        rig_(std::move(rig)) {
    scenario_id_ = sc_.at("id").get<std::string>();
    spec_id_ = util::json_has_string(sc_, "spec_id") ? sc_.at("spec_id").get<std::string>() : scenario_id_;
  }

  int run() {
    if (!sc_.contains("steps") || !sc_["steps"].is_array() || sc_["steps"].empty()) {
      emit_scenario_start();
      emit_scenario_end(false, "missing or empty steps array");
      return 1;
    }

    emit_scenario_start();

    server_ = rclcpp_action::create_server<Fibonacci>(
        node_, action_name_,
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Fibonacci::Goal> goal) {
          return rig_->on_goal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>& handle) {
          return rig_->on_cancel(handle);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>& handle) {
          // We can't map “scenario goal_id” here (ROS generates UUIDs). Mapping happens in send_goal op
          // by linking client handle -> UUID -> server handle after acceptance.
          rig_->on_accepted(handle);
          // Store latest accepted handle by UUID string for later resolution.
          last_server_handle_ = handle;
        });

    client_ = rclcpp_action::create_client<Fibonacci>(node_, action_name_);

    SpinThread spinner(node_);
    spinner.start();

    bool ok = client_->wait_for_action_server(std::chrono::seconds(2));
    if (!ok) {
      spinner.stop();
      emit_scenario_end(false, "action server not available");
      return 1;
    }

    build_dispatch();

    for (const auto& step : sc_["steps"]) {
      if (!step.contains("op") || !step["op"].is_string()) {
        emit_assert(false, step, json::object({{"note", "step missing op"}}));
        ok = false;
        break;
      }

      const std::string op = step["op"].get<std::string>();
      const auto it = dispatch_.find(op);
      if (it == dispatch_.end()) {
        emit_assert(false, step, json::object({{"note", "unknown op"}, {"op", op}}));
        ok = false;
        break;
      }

      if (!it->second(step)) {
        ok = false;
        break;
      }
    }

    spinner.stop();
    emit_scenario_end(ok, ok ? "" : "scenario failed");
    return ok ? 0 : 1;
  }

 private:
  using OpHandler = std::function<bool(const json&)>;

  void build_dispatch() {
    dispatch_.emplace("send_goal", [this](const json& step) { return op_send_goal(step); });
    dispatch_.emplace("request_cancel", [this](const json& step) { return op_request_cancel(step); });
    dispatch_.emplace("complete_terminal", [this](const json& step) { return op_complete_terminal(step); });
    dispatch_.emplace("attempt_terminal_override",
                      [this](const json& step) { return op_attempt_terminal_override(step); });
    dispatch_.emplace("wait", [this](const json& step) { return op_wait(step); });
    dispatch_.emplace("assert", [this](const json& step) { return op_assert(step); });
  }

  void emit_scenario_start() {
    auto ev = util::base_event(cfg_.version, cfg_.run_id, "scenario_start", scenario_id_);
    ev["detail"] = json::object({{"source", cfg_.scenarios_path}, {"spec_id", spec_id_}, {"backend", "ros"}});
    trace_.write(ev);
  }

  void emit_scenario_end(bool ok, const std::string& reason) {
    auto ev = util::base_event(cfg_.version, cfg_.run_id, "scenario_end", scenario_id_);
    ev["detail"] = json::object({{"ok", ok}, {"spec_id", spec_id_}});
    if (!ok && !reason.empty()) ev["detail"]["reason"] = reason;
    trace_.write(ev);
  }

  void emit_assert(bool ok, const json& expected, const json& observed) {
    auto as = util::base_event(cfg_.version, cfg_.run_id, "assertion", scenario_id_);
    as["detail"] = json::object({
        {"ok", ok},
        {"expected", expected},
        {"observed", observed},
        {"spec_id", spec_id_},
    });
    trace_.write(as);
  }

  bool op_send_goal(const json& step) {
    if (!util::json_has_string(step, "goal_id")) {
      emit_assert(false, step, json::object({{"note", "send_goal missing goal_id"}}));
      return false;
    }
    const std::string goal_id = step["goal_id"].get<std::string>();

    Fibonacci::Goal g;
    g.order = 1;

    auto fut = client_->async_send_goal(g, rclcpp_action::Client<Fibonacci>::SendGoalOptions{});
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      emit_assert(false, step, json::object({{"note", "send_goal timed out"}}));
      return false;
    }

    auto gh = fut.get();
    const bool accepted = static_cast<bool>(gh);

    auto ev = util::base_event(cfg_.version, cfg_.run_id, "goal_send_decision", scenario_id_);
    ev["goal_id"] = goal_id;
    ev["accepted"] = accepted;
    if (!accepted) ev["reason"] = "ConcurrencyPolicy";
    ev["detail"] = json::object({{"spec_id", spec_id_}});
    trace_.write(ev);

    if (!accepted) return true;

    // Map scenario goal label -> client handle.
    st_.client_handles[goal_id] = gh;

    // Best-effort map to server handle: after acceptance, server callback should have been invoked.
    // We capture last_server_handle_ and bind it to this goal label.
    if (last_server_handle_) {
      st_.server_handles[goal_id] = last_server_handle_;
    }

    return true;
  }

  bool op_request_cancel(const json& step) {
    if (!util::json_has_string(step, "goal_id")) {
      emit_assert(false, step, json::object({{"note", "request_cancel missing goal_id"}}));
      return false;
    }
    const std::string goal_id = step["goal_id"].get<std::string>();

    auto req = util::base_event(cfg_.version, cfg_.run_id, "cancel_request", scenario_id_);
    req["goal_id"] = goal_id;
    req["detail"] = json::object({{"spec_id", spec_id_}});
    trace_.write(req);

    const auto it = st_.client_handles.find(goal_id);
    if (it == st_.client_handles.end()) {
      auto resp = util::base_event(cfg_.version, cfg_.run_id, "cancel_response", scenario_id_);
      resp["goal_id"] = goal_id;
      resp["accepted"] = false;
      resp["reason"] = "UnknownGoal";
      resp["detail"] = json::object({{"spec_id", spec_id_}});
      trace_.write(resp);
      return true;
    }

    auto fut = client_->async_cancel_goal(it->second);
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      emit_assert(false, step, json::object({{"note", "cancel timed out"}}));
      return false;
    }

    const auto resp_msg = fut.get();
    const bool accepted =
        resp_msg && (resp_msg->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE);

    auto resp = util::base_event(cfg_.version, cfg_.run_id, "cancel_response", scenario_id_);
    resp["goal_id"] = goal_id;
    resp["accepted"] = accepted;
    if (!accepted) resp["reason"] = "CancelRejected";
    resp["detail"] = json::object({{"spec_id", spec_id_}});
    trace_.write(resp);

    return true;
  }

  bool op_complete_terminal(const json& step) {
    if (!util::json_has_string(step, "goal_id") || !util::json_has_string(step, "status")) {
      emit_assert(false, step, json::object({{"note", "complete_terminal missing goal_id/status"}}));
      return false;
    }

    const std::string goal_id = step["goal_id"].get<std::string>();
    const std::string status = step["status"].get<std::string>();

    const auto it = st_.server_handles.find(goal_id);
    if (it == st_.server_handles.end() || !it->second) {
      emit_assert(false, step, json::object({{"note", "server handle not found"}, {"goal_id", goal_id}}));
      return false;
    }

    Fibonacci::Result result;
    result.sequence = {1};
    auto res = std::make_shared<Fibonacci::Result>(result);

    if (status == "SUCCEEDED") it->second->succeed(res);
    else if (status == "ABORTED") it->second->abort(res);
    else if (status == "CANCELED") it->second->canceled(res);
    else {
      emit_assert(false, step, json::object({{"note", "invalid terminal status"}, {"status", status}}));
      return false;
    }

    auto ev = util::base_event(cfg_.version, cfg_.run_id, "terminal_result", scenario_id_);
    ev["goal_id"] = goal_id;
    ev["status"] = status;
    ev["detail"] = json::object({{"spec_id", spec_id_}});
    trace_.write(ev);

    return true;
  }

  bool op_attempt_terminal_override(const json& step) {
    if (!util::json_has_string(step, "goal_id")) {
      emit_assert(false, step, json::object({{"note", "attempt_terminal_override missing goal_id"}}));
      return false;
    }

    const std::string goal_id = step["goal_id"].get<std::string>();
    const auto it = st_.server_handles.find(goal_id);
    if (it == st_.server_handles.end() || !it->second) {
      emit_assert(false, step, json::object({{"note", "server handle not found"}, {"goal_id", goal_id}}));
      return false;
    }

    Fibonacci::Result result;
    result.sequence = {1};
    auto res = std::make_shared<Fibonacci::Result>(result);

    bool allowed = true;
    try {
      it->second->abort(res);
    } catch (...) {
      allowed = false;
    }

    auto ev = util::base_event(cfg_.version, cfg_.run_id, "terminal_set_attempt", scenario_id_);
    ev["goal_id"] = goal_id;
    ev["attempt"] = step.value("attempt", 2);
    ev["allowed"] = allowed;
    if (!allowed) ev["reason"] = "terminal_immutable";
    ev["detail"] = json::object({{"spec_id", spec_id_}});
    trace_.write(ev);

    return true;
  }

  bool op_wait(const json& step) {
    const int ms = step.value("ms", 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return true;
  }

  bool op_assert(const json& step) {
    const json expected = step.value("assert", json::object());
    const std::string type = expected.value("type", "");
    const std::string goal_id = expected.value("goal_id", "");

    if (type.empty()) {
      emit_assert(false, expected, json::object({{"note", "assert missing type"}}));
      return false;
    }

    json observed = goal_id.empty() ? trace_.last_for(type) : trace_.last_for(type, goal_id);
    if (observed.is_null() || observed.empty()) {
      emit_assert(false, expected, json::object({{"note", "event not found"}, {"type", type}, {"goal_id", goal_id}}));
      return false;
    }

    bool match = true;
    for (auto it = expected.begin(); it != expected.end(); ++it) {
      if (it.key() == "type") continue;
      if (!observed.contains(it.key()) || observed[it.key()] != it.value()) {
        match = false;
        break;
      }
    }

    emit_assert(match, expected, observed);
    return match;
  }

  std::ofstream& out_;
  TraceWriter trace_;
  const RunConfig& cfg_;
  const json& sc_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ActionRig> rig_;

  std::string scenario_id_;
  std::string spec_id_;
  const std::string action_name_{"oracle_action"};

  RosState st_;
  std::shared_ptr<rclcpp_action::Server<Fibonacci>> server_;
  std::shared_ptr<rclcpp_action::Client<Fibonacci>> client_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> last_server_handle_;
  std::unordered_map<std::string, OpHandler> dispatch_;
};

}  // namespace

int run_oracle_ros(const RunConfig& cfg) {
  std::ofstream trace(cfg.trace_path, std::ios::out | std::ios::trunc);
  if (!trace) {
    std::cerr << "failed to open trace file: " << cfg.trace_path << "\n";
    return 2;
  }

  // run_start (auditable)
  {
    auto ev = util::base_event(cfg.version, cfg.run_id, "run_start", "_run");
    ev["detail"] = json::object({
        {"backend", "ros"},
        {"ros_distro", util::getenv_or("ROS_DISTRO", "unknown")},
        {"rmw_implementation", util::getenv_or("RMW_IMPLEMENTATION", "default")},
    });
    util::write_event(trace, ev);
  }

  json root;
  try {
    std::ifstream in(cfg.scenarios_path);
    if (!in) throw std::runtime_error("failed to open JSON file: " + cfg.scenarios_path);
    in >> root;
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
    if (!util::json_has_string(sc, "id")) {
      std::cerr << "scenario missing id\n";
      return 2;
    }

    const std::string id = sc["id"].get<std::string>();
    std::shared_ptr<ActionRig> rig =
        (id == "A01_no_ghosts_after_rejection") ? std::make_shared<ConcurrencyGateRig>()
                                               : std::make_shared<SimpleRig>();

    RosScenarioRunner runner(trace, cfg, sc, node, std::move(rig));
    if (runner.run() != 0) rc = 1;
  }

  return rc;
}

}  // namespace oracle
