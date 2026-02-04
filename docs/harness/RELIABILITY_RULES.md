# Reliability Rules for Oracle Harness

## Evidence Validity Gates

Evidence artifacts must pass these gates before results are trusted:

| Artifact | Gate |
|----------|------|
| `trace.jsonl` | Exists, non-empty (>=10 bytes), valid JSONL (each line parses) |
| `report.json` | Exists, valid JSON, contains `scenarios` object with >=1 entry, each scenario has `outcome` |

**Failure Classification:**
- `INFRA_FAIL` — evidence gate failed (missing/empty/malformed artifacts)
- `SEMANTIC_FAIL` — evidence valid, scenario outcome is FAIL

Never trust a result without verifiable evidence. An `INFRA_FAIL` is not a pass; it blocks the pipeline.

## Repeatability Rules

### 5-Run Protocol
Run every scenario 5 times minimum. All 5 runs must produce identical outcomes for the scenario to be considered stable.

### Evidence Separation
Each run must write to a separate evidence directory:
```
evidence/<distro>/<backend>/<scenario>/run_<N>/
├── trace.jsonl
├── report.json
└── stdout.log
```

Never overwrite evidence from previous runs. The aggregation layer reads all runs.

### Build-Once Pattern
Build container images once per configuration, then execute multiple runs against the same image. Avoid rebuilding between iterations.

## Exit Code Semantics

### oracle_core (Rust)
| Code | Meaning |
|------|---------|
| 0 | All scenarios PASS |
| 1 | One or more scenarios FAIL (semantic failure) |
| 2 | Bundle error (invalid JSON, unsupported op, schema violation) |
| 4 | System error (I/O failure, crash) |

### Backends (backend_prod, etc.)
| Code | Meaning |
|------|---------|
| 0 | Execution completed (wrote trace) |
| 2 | Bundle error (unsupported op, schema violation) |
| 4 | System error (crash, ROS failure) |

**Backends never exit 1.** Pass/fail is determined by the oracle from the trace, not by the backend.

## No-Cheating Constraints

### Trace Schema Lock
Only emit approved event types from `harness/schemas/trace_event.schema.json`:
- `run_start`, `run_end`
- `backend_capabilities`
- `scenario_start`, `scenario_end`
- `op_start`, `op_end`
- `goal_send`, `goal_response`, `goal_cancel`, `goal_status`, `goal_result`
- `feedback`, `diagnostic`

Forbidden fields in any event: `ok`, `pass`, `fail`, `verdict`, `assertion`.
Operation outcomes belong in `op_end.detail`.

### Strict Fidelity
Backends must call real ROS 2 client library APIs. No mocking, no hard-coded traces, no simulated accept/reject behavior.

Example violation: Using a custom lock to make concurrent transitions appear sequential.
Example compliance: Calling `rclcpp_lifecycle::LifecycleNode::configure()` and observing real state machine behavior.

### Capability Declaration
Backends must accurately report capabilities in `backend_capabilities`. The harness may skip scenarios requiring unsupported capabilities, but backends must not fake support.

## Aggregation Requirements

The aggregation layer must:
1. Apply validity gates to each run independently
2. Count `PASS`, `SEMANTIC_FAIL`, `INFRA_FAIL` per cell
3. Mark a cell as:
   - `PASS` — all runs passed validity gate and scenario outcome = PASS
   - `FAIL` — any run has semantic failure
   - `FLAKY` — mixed results (some pass, some fail)
   - `INFRA_FAIL` — all runs failed validity gates

Output: `evidence/aggregate.json` (machine-readable) + `evidence/aggregate.md` (human-readable table).
