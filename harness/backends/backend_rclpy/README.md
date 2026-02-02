# backend_rclpy

Minimal Python/rclpy backend for Oracle Harness probe matrix.

## Purpose

Probe-quality backend implementing ops needed for L05, S11, P04 scenarios:
- **L05**: Concurrent lifecycle transition rejection
- **S11**: Lifecycle-executor safety (responsiveness during transitions)
- **P04**: Atomic batch parameter updates

## Architecture

```
backend_rclpy.py       # Main entry point (CLI: <bundle> <trace>)
trace_writer.py        # JSONL trace emission (oracle schema)
ops/
  lifecycle.py         # Lifecycle ops (change_state, get_state)
  params.py            # Parameter ops (declare, set_batch)
```

## Supported Operations

- `init_lifecycle_node`: Initialize LifecycleNode with MultiThreadedExecutor
- `change_state`: Synchronous lifecycle transition
- `change_state_async`: Async transition with artificial delay (for L05 concurrency)
- `get_state`: Query current state (for S11 responsiveness)
- `declare_param`: Declare parameter
- `set_params_batch`: Atomic batch parameter set (for P04)
- `wait`: Sleep for specified milliseconds

## Usage

```bash
python3 backend_rclpy.py <bundle_path> <trace_path>
```

## Exit Codes

- **0**: Success
- **2**: Bundle error (malformed JSON, missing ops)
- **4**: System error (ROS failure, I/O error)

## Implementation Notes

- Uses `rclpy.lifecycle.LifecycleNode` for lifecycle support
- `MultiThreadedExecutor` for S11 responsiveness testing
- Threading for async operations to force concurrency windows
- Emits oracle-compliant JSONL trace events

## Limitations (Probe Quality)

- Minimal error handling
- No report.json generation (core handles that)
- Simplified state machine (only supports probe scenario paths)
- No production-grade logging or diagnostics
