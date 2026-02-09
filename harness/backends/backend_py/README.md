# backend_py: Python Backend for ROS 2 Semantic Contracts

Python backend using rclpy for H00/H01 parity with backend_prod.

## Architecture

- **TraceWriter**: Thread-safe JSONL emission with envelope fields (version, run_id, t_ns, sequence)
- **Operation Dispatch**: Registry pattern mapping op names to handler functions
- **BackendState**: Per-scenario ROS node and action client management
- **Exit Codes**: 0 (success), 2 (BundleError), 4 (SystemError)

## Build & Install

```bash
cd harness/backends/backend_py
pip3 install -e .
```

## Usage

### Direct Invocation

```bash
backend_py <bundle_path> <trace_path>
```

### Via Harness

```bash
ORACLE_BACKEND=py ./harness/scripts/run_harness.sh \
  harness/scenarios/scenarios_H.json \
  /tmp/trace.jsonl \
  /tmp/report.json
```

### Docker

```bash
ORACLE_BACKEND=py docker compose up --abort-on-container-exit
```

## Supported Operations

- `get_clock` - ROS clock API (H01)
- `send_goal` - Action client send_goal (H00)
- `wait` - Sleep for specified milliseconds

## Capabilities

- `actions.basic` - send_goal support
- `clock.ros_time` - ROS time API

## Environment Variables

- `BACKEND_PY_RUN_ID` - Override run_id (default: "run_py")
