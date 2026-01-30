# Stub Backend Contract (v0.1)

This document defines the required behaviour of the **stub backend**.

The stub backend is a deterministic event emitter used to validate the oracle harness
in isolation from any external system.

It exists to prove:
- scenario parsing and execution
- append-only JSONL trace generation
- trace schema compliance
- core-side expectation evaluation
- reporting and exit semantics

It does **not** encode or validate domain semantics.

---

## Invariants

- The backend emits **observations only**.
- The trace is **append-only JSONL** (one JSON object per line).
- Backend exit codes indicate **execution success only**, not semantic correctness.

---

## Supported scope

The backend must support the scenarios defined in:

```
harness/scenarios/scenarios_H.json
```

Additional scenario support may be added incrementally.

---

## Supported operations

### `send_goal`

Required fields:
- `goal_id: string`
- `payload.mode: "accept" | "reject"`

Behaviour:
- Emit a `goal_send` event.
- Emit a `goal_response` event.
- `goal_response.accepted` is determined solely by `payload.mode`.

No additional behaviour is required.

---

### `wait`

Required fields:
- `ms: integer ≥ 0`

Behaviour:
- Delay execution for approximately `ms` milliseconds.
- No semantic events are required.

---

## Required trace events

All events must conform to:

```

harness/schemas/trace_event.schema.json

```

For each scenario execution, the backend must emit:

1. `scenario_start`
2. For each operation (in order):
   - `op_start` (recommended)
   - zero or more operation-specific events
   - `op_end` (recommended)
3. `scenario_end`

For each `send_goal` operation, the backend must emit:
- `goal_send`
- `goal_response`

---

## Sequencing requirements

- Events must be written in execution order.
- `t_ns` must be monotonic non-decreasing.
- If present, `sequence` must be monotonic increasing.

---

## Prohibited behaviour

The backend must not:
- emit assertions or verdicts
- emit pass/fail indicators
- infer or encode correctness
- modify previously written trace data
