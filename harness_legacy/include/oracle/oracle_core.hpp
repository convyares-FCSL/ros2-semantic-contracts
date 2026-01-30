#pragma once

#include <string>

namespace oracle {

enum class BackendKind {
  Stub,
  Ros
};

struct RunConfig {
  std::string scenarios_path;
  std::string trace_path;

  // Keep stable; later can be CLI args.
  std::string version{"0.1"};
  std::string run_id{"dev"};
};

/// Run the oracle over a scenarios JSON file and emit trace JSONL.
/// Returns:
///  - 0: runner executed (pass/fail is represented in trace; audit is separate)
///  - 2: usage / IO / schema errors (no meaningful trace)
int run_oracle(const RunConfig& cfg, BackendKind backend);

}  // namespace oracle
