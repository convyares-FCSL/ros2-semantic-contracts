#include "oracle/oracle_core.hpp"

#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: oracle_a <scenarios.json> <trace.jsonl>\n";
    return 2;
  }

  oracle::RunConfig cfg;
  cfg.scenarios_path = argv[1];
  cfg.trace_path = argv[2];

  return oracle::run_oracle(cfg, oracle::BackendKind::Stub);
}
