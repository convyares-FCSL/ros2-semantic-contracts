#pragma once

#include "oracle/oracle_core.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

namespace oracle::ros {

using Json = nlohmann::json;
using ActionT = example_interfaces::action::Fibonacci;
using ClientHandle = rclcpp_action::ClientGoalHandle<ActionT>;
using ServerHandle = rclcpp_action::ServerGoalHandle<ActionT>;

struct ScenarioContext {
  std::string scenario_id;
  std::string spec_id;
  std::string action_name{"oracle_action"};
};

struct RosState {
  std::unordered_map<std::string, ClientHandle::SharedPtr> client_by_label;
  std::unordered_map<std::string, std::shared_ptr<ServerHandle>> server_by_label;
  std::shared_ptr<ServerHandle> last_server_handle;
};

class SpinWorker {
 public:
  explicit SpinWorker(rclcpp::Node::SharedPtr node);
  ~SpinWorker();

  void start();
  void stop();

 private:
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> running_{false};
  std::thread th_;
};

class IActionRig {
 public:
  virtual ~IActionRig() = default;

  virtual rclcpp_action::GoalResponse on_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ActionT::Goal> goal) = 0;

  virtual rclcpp_action::CancelResponse on_cancel(
      const std::shared_ptr<ServerHandle>& handle);

  virtual void on_accepted(const std::shared_ptr<ServerHandle>& handle, RosState& st);
};

class ConcurrencyGateRig final : public IActionRig {
 public:
  rclcpp_action::GoalResponse on_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ActionT::Goal> goal) override;

 private:
  std::atomic<bool> accepted_one_{false};
};

class SimpleRig final : public IActionRig {
 public:
  rclcpp_action::GoalResponse on_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ActionT::Goal> goal) override;
};

class OpsRunner {
 public:
  OpsRunner(std::ofstream& out,
            const RunConfig& cfg,
            const Json& scenario,
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<IActionRig> rig);

  int run();

 private:
  using OpFn = std::function<bool(const Json&)>;

  void build_dispatch();
  void emit_scenario_start();
  void emit_scenario_end(bool ok, const std::string& reason);
  void emit_assert(bool ok, const Json& expected, const Json& observed);

  bool op_send_goal(const Json& step);
  bool op_request_cancel(const Json& step);
  bool op_complete_terminal(const Json& step);
  bool op_attempt_terminal_override(const Json& step);
  bool op_wait(const Json& step);
  bool op_assert(const Json& step);

  std::ofstream& out_;
  const RunConfig& cfg_;
  Json sc_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<IActionRig> rig_;

  ScenarioContext ctx_;
  RosState st_;

  std::shared_ptr<rclcpp_action::Server<ActionT>> server_;
  std::shared_ptr<rclcpp_action::Client<ActionT>> client_;
  std::unordered_map<std::string, OpFn> dispatch_;

  bool ok_{true};
  std::string reason_;
};

std::shared_ptr<IActionRig> select_rig_for_scenario(const std::string& scenario_id);

}  // namespace oracle::ros
