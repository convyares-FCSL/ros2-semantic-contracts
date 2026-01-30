# Backend Contract (Oracle Harness) — v0.1-min

This document defines the stable interface between the **core harness runner** and any **backend**.

Backends are adapters: they execute operations against a target system and emit observations.
The core runner is the judge: it evaluates expectations over the trace and produces results.

---

## 1. Roles (non-negotiable)

### Core runner (authoritative)
- Loads scenario bundle(s).
- Selects and executes a backend.
- Validates trace events against the trace schema.
- Evaluates expectations over the produced trace.
- Produces reports and PASS/FAIL verdicts.

### Backend (non-authoritative)
- Executes `ops` against a target system.
- Emits observations as JSONL trace events.
- May emit diagnostics as JSONL trace events.
- Must never emit PASS/FAIL, assertions, or verdict signals.

---

## 2. Trace is append-only JSONL

- The backend writes **one JSON object per line**.
- The backend only appends. No rewriting, truncating, or post-processing.
- Events are written in execution order.

---

## 3. Exit codes are transport/execution-level only

Backend exit codes reflect ability to run and produce observations, not semantic correctness:

- `0` = executed successfully (trace written)
- `2` = usage / config / IO failure (no meaningful run)
- `3` = target unavailable (cannot connect/start required services)
- `4` = backend internal error

The core runner decides PASS/FAIL using trace content.

---

## 4. Invocation

Backends must support:

- `backend_<name> <scenario_bundle.json> <trace.jsonl>`

Where:
- `<scenario_bundle.json>` is a scenario bundle file
- `<trace.jsonl>` is the output path (created if missing; append-only)

Optional environment variables:
- `RUN_ID` (string): propagated into trace events
- `VERSION` (string): propagated into trace events
- Target-specific environment variables may be recorded via `env` events.

---

## 5. Scenario bundle expectations

Scenario bundles are declarative and contain:

- `group` (string)
- `scenarios` (object keyed by `scenario_id`)

Each scenario contains:
- `ops` (array): executed by the backend
- `expects` (array): evaluated by the core

Backends must:
- Execute only `ops` they support.
- Ignore `expects` entirely.
- Exit with code `2` if required fields for an op are missing.
- Emit `scenario_start` and `scenario_end` for each scenario attempted.

---

## 6. Trace event minimum fields

Each emitted event is a JSON object with top-level keys:

Required:
- `version` (string)
- `run_id` (string)
- `t_ns` (uint64): monotonic timestamp in nanoseconds
- `type` (string): event type (core vocabulary)
- `scenario_id` (string)

Optional:
- `detail` (object): additional structured metadata
- `goal_id` (string): scenario-local goal label

The core runner enforces the trace schema.

---

## 7. Required lifecycle events

Backends must emit:

### `run_start`
Once per run, before any scenario activity.
- `scenario_id = "_run"`
- `detail.backend` = backend identifier

### `scenario_start`
Once per scenario, before executing its ops.
- `scenario_id` = scenario identifier
- Optional: `detail.source`, `detail.spec_id`, `detail.backend`

### `scenario_end`
Once per scenario, after executing its ops.
- `scenario_id` = scenario identifier
- Optional: `detail.error` (object) if the backend encountered execution failure

### `run_end` (recommended)
Once per run after all scenarios.
- `scenario_id = "_run"`
- Optional: `detail.error` (object) if the backend encountered execution failure

---

## 8. Core trace vocabulary (v0.1-min)

The core runner understands only these event `type` values:

- `run_start`, `run_end`
- `scenario_start`, `scenario_end`
- `env`, `diagnostic`
- `op_start`, `op_end`
- `goal_send`, `goal_response`
- `status`, `result`

Additional event types are not part of the core contract and must not be required
by core expectations until the contract is extended.

Required fields by type (minimum):

- `goal_send`: `goal_id`
- `goal_response`: `goal_id`, `accepted` (boolean), optional `reason` (string)
- `status`: `goal_id`, `state` (string), optional `sequence` (uint)
- `result`: `goal_id`, `state` (string)

---

## 9. Ops (executed by backends)

Backends may implement a subset of ops.
If a scenario requires an unsupported op, the backend must:
- emit a `diagnostic` event describing the unsupported op
- and exit non-zero (or include an error in `scenario_end.detail.error`)

Core v0 ops:
- `send_goal(goal_id, payload)`
- `wait(ms)`
- `request_cancel(goal_id)` (reserved; not required for v0.1-min)
- `set_param`, `get_param`, `call_service` (reserved)

---

## 10. Expectations (evaluated by core)

Scenarios may include expectations such as:
- event existence
- field match
- ordering constraints
- timing windows

Backends ignore expectations.

---

## 11. Versioning

- Trace events include `version`.
- Contract changes that affect trace or scenario shapes must be versioned.
- The core runner must not depend on event types outside the core vocabulary unless the contract is extended.
