#include "oracle/oracle_core.hpp"

#include <iostream>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: oracle_a_ros <scenarios.json> <trace.jsonl>\n";
    return 2;
  }

  rclcpp::init(argc, argv);

  oracle::RunConfig cfg;
  cfg.scenarios_path = argv[1];
  cfg.trace_path = argv[2];

  // For now, still stubbed core behaviour; next phase wires real ROS calls.
  int rc = oracle::run_oracle(cfg, oracle::BackendKind::Ros);

  rclcpp::shutdown();
  return rc;
}
