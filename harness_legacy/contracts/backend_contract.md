# Backend Contract (Oracle Harness)

This document defines the stable interface between the **core harness runner** and any **backend**.
Backends are *adapters*: they execute operations against a target stack and emit *observations*.
The core runner is the *judge*: it evaluates expectations over the produced trace and produces PASS/FAIL.

## 1. Roles and non-negotiables

### Core runner (authoritative)
- Loads scenario JSON.
- Selects and executes a backend.
- Validates trace schema.
- Evaluates **expectations** over the trace.
- Produces verdicts (PASS/FAIL) and reports.

### Backend (non-authoritative)
- Executes ops (send goal, cancel, wait, etc.) against a target.
- Emits **observations** as JSONL trace events.
- May emit **diagnostics** (notes, debug context) as trace events.
- Must never emit PASS/FAIL or make assertions.

### Hard rule: trace is append-only
- Backends only append JSON objects, one per line.
- No rewriting, truncating, or post-processing the trace.
- If a backend fails mid-run, it should still have emitted enough context for audit.

### Hard rule: backend exit code is transport-level only
Backends return exit codes based on ability to run/connect, not test semantics:
- `0` = executed successfully (observations written; test verdict is core responsibility)
- `2` = usage/config/IO failure (no meaningful run possible)
- `3` = target unavailable (could not connect/start required services)
- `4` = backend internal error (unexpected exception, invariant broken in backend code)

Core runner decides PASS/FAIL using the trace content.

## 2. Invocation

Backends must support this CLI:

- `backend_<name> <scenarios.json> <trace.jsonl>`

Where:
- `<scenarios.json>` is the full scenario file (contains `scenarios: [...]`)
- `<trace.jsonl>` is the output file path to write JSONL events

Environment variables (optional but recommended):
- `RUN_ID` (string): propagated into trace events
- `VERSION` (string): propagated into trace events
- `ROS_DISTRO`, `RMW_IMPLEMENTATION`, etc. may be recorded by ROS-based backends

## 3. Scenario format expectations

Scenarios are declarative. They contain:
- `id` (string) e.g. `A01_no_ghosts_after_rejection`
- `spec_id` (optional string)
- `steps` (array)

Steps are either:
- **ops**: executed by the backend
- **expects**: evaluated by the core runner over the trace

Backends must:
- Execute only the ops they support.
- Ignore unknown `expect` sections (because the core evaluates them).
- Fail with exit code `2` if required fields are missing.
- Emit a `scenario_start` and `scenario_end` for every scenario attempted.

## 4. Trace schema: minimum required fields

Each emitted event is a JSON object with these top-level keys:

- `version` (string)
- `run_id` (string)
- `t_ns` (uint64): monotonic timestamp in nanoseconds
- `type` (string): event type
- `scenario_id` (string)

Optional (but strongly recommended):
- `detail` (object): additional structured metadata
- `goal_id` (string): scenario logical label for a goal (not necessarily a ROS UUID)

The core runner will enforce the trace schema (e.g., `trace_schema.json`).

## 5. Required lifecycle events

Backends must emit:

### run_start
Emitted once per run, before any scenario activity.
- `type = "run_start"`
- `scenario_id = "_run"`
- `detail.backend` = backend identifier (e.g., `stub`, `ros_rclcpp`)
- Optional: environment metadata (ros distro, rmw implementation, container image tag, git sha)

### scenario_start
Emitted once per scenario, before executing scenario steps.
- `type = "scenario_start"`
- `detail.source` = scenarios file path
- `detail.spec_id` = spec id if present
- `detail.backend` = backend id

### scenario_end
Emitted once per scenario, after executing scenario steps.
- `type = "scenario_end"`
- `detail.ok` = boolean *execution* success (not spec pass/fail)
- If `detail.ok=false`, `detail.reason` should explain backend execution failure

### run_end (recommended)
Emitted once per run after all scenarios.
- `type = "run_end"`
- `scenario_id = "_run"`
- `detail.ok` = boolean *execution* success for the backend run

## 6. Standard observation event types (v0 set)

Backends should prefer these types and fields so expectations remain stable across stacks.

### goal_send_decision
Observation: result of sending a goal.
- `type = "goal_send_decision"`
- `goal_id` = scenario label
- `accepted` = boolean
- Optional `reason` when rejected (e.g., `ConcurrencyPolicy`)

### cancel_request
Observation: cancel requested.
- `type = "cancel_request"`
- `goal_id`

### cancel_response
Observation: cancel response.
- `type = "cancel_response"`
- `goal_id`
- `accepted` = boolean
- Optional `reason` (e.g., `UnknownGoal`, `CancelRejected`)

### terminal_result
Observation: terminal result recorded.
- `type = "terminal_result"`
- `goal_id`
- `status` = `"SUCCEEDED" | "ABORTED" | "CANCELED"`

### terminal_set_attempt
Diagnostic observation: an attempted illegal terminal transition.
- `type = "terminal_set_attempt"`
- `goal_id`
- `attempt` = integer (e.g., 2)
- `allowed` = boolean
- Optional `reason` (e.g., `terminal_immutable`)

### diagnostic (optional)
Backend-only debug events, never used for pass/fail.
- `type = "diagnostic"`
- `detail` contains structured fields (`note`, `context`, `exception`, etc.)

## 7. Ops (executed by backends)

Backends are expected to implement an op subset. Unknown ops:
- If required for the scenario, backend should mark scenario execution failed and emit `scenario_end ok=false`.

Core v0 ops:
- `send_goal(goal_id, ...)`
- `request_cancel(goal_id)`
- `wait(ms)`
- `complete_terminal(goal_id, status)` (only if backend can control terminalization)
- `attempt_terminal_override(goal_id, attempt, status)` (backend-defined, diagnostic)

## 8. Expectations (evaluated by core)

Scenarios may include `expect` steps such as:
- expected event exists
- expected fields match
- ordering constraints
- timing windows / timeouts

Backends must ignore these beyond emitting sufficient observations.

## 9. Versioning
- Trace events include `version`.
- Changes to event shapes must be backward compatible within a major version.
- New event types are allowed; core runner may ignore unknown types unless referenced by expectations.

