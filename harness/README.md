# Oracle Harness (Phase 8)

This harness validates the repository’s semantic contracts against observed ROS 2 behaviour.

It is designed as an **oracle + audit pipeline**:
- the oracle **executes scripted semantic scenarios** and emits a machine-readable trace
- the auditor **consumes the trace** and produces authoritative pass/fail verdicts

The trace is the source of truth.

---

## Sequencing (C: A → B)

### A) Mechanistic oracle
Goal: validate core action semantics with minimal confounders.

Two backends are supported:

#### Stub backend (default)
- No ROS runtime required
- Deterministic, scripted behaviour
- Used to validate:
  - scenario structure
  - assertion logic
  - trace format
  - audit pipeline
- Acts as the semantic “golden reference” for later comparison

#### ROS backend (`rclcpp`)
- Single process
- Uses runtime ROS environment (no mocks)
- Observes real client/server behaviour
- Environment is captured from runtime variables:
  - `ROS_DISTRO`
  - `RMW_IMPLEMENTATION`

Current semantic coverage (A1):
- send_goal accept/reject (“no ghosts”)
- cancel rejection for unknown goals
- deterministic decision envelopes

Additional A-series specs will progressively replace stub logic with observed ROS behaviour.

Outputs:
- machine-readable trace (JSONL) with timestamps and event types
- summarized verdicts per scenario (pass/fail)

---

### B) System oracle (Nav2) — planned
Goal: confirm core semantics are not contradicted when orchestration and policy layers exist.

Planned characteristics:
- Minimal Nav2 bringup (containerized)
- Indirect exercise of actions via Nav2 stacks
- Reuse the same trace + audit machinery
- Explicitly record “expected divergence” where Nav2 policy/executor behaviour is out of scope for core semantics

---

## Reproducibility and dependencies

This harness vendors third-party headers (e.g. `nlohmann/json`) under `include/third_party`
to ensure:
- reproducible builds
- offline operation
- audit-safe provenance

No network access is required to build or run the harness.

---

## Quickstart

### Build
```bash
cmake -S . -B build
cmake --build build
````

### Run (stub backend)

```bash
./run_harness.sh
```

### Run (ROS backend)

```bash
ORACLE_BACKEND=ros ./run_harness.sh
```

The auditor will print:

* which backend was used
* ROS distro / RMW (if applicable)
* per-scenario verdicts
* overall summary

The trace location is printed at the end of the run.

---

## Directory layout

* `include/`

  * `oracle/`         transport-agnostic oracle interfaces
  * `third_party/`    vendored headers (pinned)
* `src/`

  * `oracle_core.cpp` core oracle logic (no ROS)
  * `oracle_a.cpp`    stub backend runner
  * `oracle_a_ros.cpp` ROS backend adapter
  * `audit_trace.cpp` trace auditor (verdict printer)
* `configs/`

  * scenario definitions
* `build/`

  * CMake build output (ignored)
* `run_harness.sh`

  * convenience runner (oracle → audit)

---

## Design principles

* **Trace-first**: the trace is the authoritative artefact
* **Separation of concerns**:

  * oracle emits facts
  * auditor decides pass/fail
* **Environment transparency**:

  * what was tested is explicitly recorded
* **Incremental realism**:

  * stub → ROS → Nav2, without changing the audit model