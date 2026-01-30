# Oracle Harness (greenfield baseline)

This folder defines the **baseline contract, assets, and executables** for the
ROS 2 Semantic Oracle Harness.

The harness exists to **observe behavior**, not to encode semantics.
All semantic judgment lives in the core runner.

---

## Non-negotiables

(See `contracts/backend_contract.md`)

- Backends emit **append-only JSONL trace events**
  - One JSON object per line
  - No rewriting, truncation, or post-processing
- Backends emit **observations only**
  - No assertions
  - No pass/fail
  - No semantic judgment
- The **core runner** is the sole authority for:
  - Trace validation
  - Expectation evaluation
  - Verdict production
- Backend exit codes are **transport-level only**
  - Backend success ≠ test pass

---

## Layout

- `contracts/`
  - Binding interface contracts between core and backends
- `schemas/`
  - JSON Schemas for scenario bundles and trace events
- `scenarios/`
  - Grouped scenario bundles (`scenarios_A.json`, `scenarios_L.json`, …)
- `backends/`
  - Backend implementations (stub, ROS adapters, production stacks)
- `core/`
  - Core runner (judge, evaluator, reporter)
- `scripts/`
  - Operator-facing convenience scripts

---

## Scenarios

Scenario bundles are **declarative assets**, not code.

They are keyed by scenario id for review ergonomics:

```json
"scenarios": {
  "A01_unique_goal_identity": {
    "spec_id": "A01",
    "ops": [],
    "expects": []
  }
}
````

* `ops`

  * Executed by the backend
  * Backend-specific, observational only
* `expects`

  * Evaluated exclusively by the core runner over the trace

Backends **must ignore** `expects`.

---

## Harness self-tests (H-series)

The `H` scenario group validates the harness itself.

Example:

* `H00_smoke_trace_roundtrip`

  * Scenario load → backend run → JSONL trace → core evaluation

These scenarios are:

* Deterministic
* Stub-only
* Not tied to ROS semantics
* Used to validate pipeline correctness before introducing real stacks

---

## Quickstart (H00)

From the repo root:

```bash
cd harness
./scripts/run_harness.sh
```

Run a different scenario bundle:

```bash
./scripts/run_harness.sh scenarios/scenarios_A.json
```

Select a backend (default is `stub`):

```bash
ORACLE_BACKEND=stub ./scripts/run_harness.sh
# ORACLE_BACKEND=ros  ./scripts/run_harness.sh   # future: rclcpp adapter
```

Outputs:

* Trace: `/tmp/oracle_trace.jsonl`
* Report: `/tmp/oracle_report.json`

A successful run prints a per-scenario summary and an overall verdict:

```
PASS H00_smoke_trace_roundtrip (H00)
ALL TESTS PASSED
```

---

## Design intent

* Prefer **deletion over extension**
* Keep backends thin and replaceable
* Keep the core strict and deterministic
* Treat traces as auditable artifacts, not transient logs
