# backend_prod

Production-grade C++20 backend for the Oracle Harness.

## Overview

`backend_prod` is a deterministic, bundle-driven backend that executes ROS 2 scenario bundles and emits JSONL trace events. It serves as the production baseline for validating ROS 2 behavioral contracts.

## Architecture

- **Input**: Scenario bundle (JSON) defining operations to execute
- **Output**: JSONL trace of observations with envelope metadata
- **State**: Ephemeral; no persistence beyond trace file

## Structure

```
include/backend_prod/          # Public API
  backend.hpp                  # run_backend() interface
  types.hpp                    # Bundle/Op/Scenario types
  trace.hpp                    # TraceWriter class
include/3rdparty/              # Vendored dependencies
  nlohmann/json.hpp            # JSON library (v3.11.3)
src/
  main.cpp                     # Entry point, ROS init/shutdown
  backend.cpp                  # Bundle parsing + execution loop
  backend.hpp                  # Internal coordination header
  ops/                         # Operation implementations
    actions.cpp                # Action ops (send_goal)
    generic.cpp                # Generic ops (wait)
    lifecycle.cpp              # Lifecycle ops (placeholder)
    params.cpp                 # Parameter ops (placeholder)
  internal/                    # Implementation details
    trace.cpp                  # TraceWriter implementation
    ros_utils.cpp              # ROS conversions (if needed)
```

## Dependencies

- **ROS 2**: `rclcpp` (Jazzy)
- **JSON**: Vendored `nlohmann/json` v3.11.3 (no apt install required)
- **C++**: C++20 standard
- **CMake**: 3.8+ (ROS 2 Jazzy standard)

## Building

```bash
# From repository root
colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage

```bash
# Direct invocation
./install/backend_prod/lib/backend_prod/backend_prod <bundle_path> <trace_path>

# Via harness script
export ORACLE_BACKEND=prod
./harness/scripts/run_harness.sh harness/scenarios/scenarios_H.json /tmp/trace.jsonl /tmp/report.json
```

## Exit Codes

- **0**: Successful execution
- **2**: Usage error (invalid args, malformed bundle, unsupported op)
- **4**: System error (I/O failure, internal error)

## Supported Operations

### H00 (Harness Smoke Test)
- `send_goal`: Emit goal_send + goal_response (driven by `payload.mode`)
- `wait`: Sleep for specified milliseconds

### Future Scenarios
- Lifecycle operations (scenario_L)
- Parameter operations (scenario_P)
- Advanced action operations (scenario_A)

## Trace Format

JSONL (one JSON object per line) with envelope:

```json
{"version":"0.1","run_id":"run_prod","t_ns":1234567,"sequence":0,"type":"run_start","scenario_id":"_run","detail":{"backend":"prod"}}
```

**Envelope fields** (always present):
- `version`: Trace schema version
- `run_id`: Backend run identifier
- `t_ns`: Nanoseconds since process start (steady_clock)
- `sequence`: Monotonic event counter

## Design Principles

1. **Bundle-driven**: All behavior driven by scenario bundle, not hardcoded
2. **Deterministic**: Predictable event ordering and timing
3. **Minimal API**: Small public surface for stability
4. **Clean boundaries**: Public headers never include `src/` internals
5. **No external deps**: Vendored JSON library for Docker compatibility

## Testing

```bash
# Local
export ORACLE_BACKEND=prod
./harness/scripts/run_harness.sh harness/scenarios/scenarios_H.json /tmp/trace.jsonl /tmp/report.json

# Docker
docker compose build
docker compose up --abort-on-container-exit
```

Expected output: `PASS H00_smoke_trace_roundtrip`
