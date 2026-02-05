# backend_prod

## 1. What backend_prod Is

`backend_prod` is a reference backend used to exercise ROS 2 semantics against the harness.

It exists to generate **externally observable evidence** via **real ROS 2 client-library APIs** and emit that evidence as schema-valid JSONL trace events for oracle evaluation.

It is **not an application**, **not a test fixture**, and **not a simulator**.

## 2. What backend_prod Is Not (Hard Prohibitions)

`backend_prod` MUST NOT:

- Host **policy-driven peers** (servers / fixtures) **when the scenario’s SUT is the client**.
- Contain logic that decides accept / reject / terminal outcomes **for scenarios where the backend is not the SUT**.
- Read scenario fields to influence peer behavior.
- Emit fabricated, advisory, or oracle-directed events.
- Emit trace event types outside `trace_event.schema.json`.

These are **architectural boundaries**, not guidelines.

## 3. SUT / Peer Boundary Rule

Each scenario defines **what is under test** (the SUT) and **what provides stimulus** (the peer).

The backend’s allowed behavior depends on that boundary.

### Client-Semantics Scenarios (e.g. Actions Axx with `sut=client`)

For scenarios validating **client-side semantics**:

- `backend_prod` is **client-only**
- Action / lifecycle / parameter servers are provided **externally by the scenario environment**
- Evidence must arise solely from:
  - real ROS 2 client APIs (`async_send_goal`, `wait_for_action_server`, callbacks)
  - graph-observable outcomes (results, status, absence of events)

`backend_prod` MUST NOT host or configure peers whose behavior can be scripted to satisfy the scenario.

This prevents **self-confirming peers**.

Reference: `docs/provenance/oracle/boundaries.md`

### Server- or Node-Semantics Scenarios (future families)

For scenarios validating **server-side or node-internal semantics** (e.g. lifecycle node correctness, parameter service behavior):

- The backend **may implement the server or node**, because it is the SUT
- The peer (client / driver) must be external
- Evidence is judged by externally observable behavior

This is allowed because the backend is not grading its own policy outcomes.

## 4. Evidence Discipline

Events are emitted **only** as a reflection of real ROS 2 API outcomes.

- `goal_send_decision` reflects actual goal negotiation completion
- `result` reflects `WrappedResult.code`, mapped canonically
- Absence of events is valid evidence and is evaluated by the oracle, not the backend

The backend never infers or declares verdicts.

## 5. Concurrency & Trace Integrity

`TraceWriter` is thread-safe.

Writes are serialized to guarantee **one complete JSONL line per event**.
This guarantees trace integrity, not semantic ordering.

Event ordering remains governed by executor scheduling and ROS runtime behavior.

## 6. Scenario Isolation Assumption

`backend_prod` does not select peers.

Peer selection and lifecycle are owned by the environment
(e.g. `launch_peer.sh`, Docker, or runner scripts).

Scenario isolation is achieved via:
- ephemeral bundles
- environment variables
- runner-level orchestration

Never backend logic.

## 7. Change Control

Any change that alters:
- backend role (client vs server)
- peer assumptions
- event semantics
- trace guarantees

must be reviewed against:

- `trace_event.schema.json`
- `docs/provenance/oracle/boundaries.md`

Violations are architectural regressions.

---

## Overview

- **Input**: Scenario bundle (JSON)
- **Output**: JSONL trace of observations
- **State**: Ephemeral; no persistence beyond the trace file

## Structure

```


## Project Structure

- `include/backend_prod/`: Public headers.
- `src/`: Core backend library and operations.
- `actors/`: Standalone executables (actors) that run alongside the backend. These are first-class siblings to `src/` and dependencies of the environment, not the backend library.

```

## Dependencies

- ROS 2 Jazzy (`rclcpp`)
- Vendored `nlohmann/json` v3.11.3
- C++20

## Exit Codes

- `0` — success
- `2` — usage / bundle error
- `4` — system / ROS error

## Trace Format

JSONL, schema-governed by `trace_event.schema.json`.

Each line is a complete, independent observation.