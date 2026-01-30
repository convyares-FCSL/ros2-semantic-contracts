# Oracle Naming Rules

This document defines naming and mapping rules for semantic oracle scenarios and their traceability
to specification invariants.

The goal is:
- stable identifiers in logs and traces
- one-to-one traceability from specs → scenarios → audit verdicts
- predictable growth as Phase A (mechanistic) expands and Phase B (Nav2/system) is added

---

## Terms

- **Spec ID**: Stable identifier for one atomic semantic invariant (e.g., A01).
- **Scenario ID**: Stable identifier for one runnable oracle scenario (e.g., A01_no_ghosts_after_rejection).
- **Oracle A**: Mechanistic oracle (rclcpp-only; minimal confounders).
- **Oracle B**: System oracle (Nav2; orchestration/policy layer present).
- **Audit**: Pass/fail verdicts computed from the trace (JSONL), not from hidden oracle state.

---

## 1) Spec ID scheme

Spec IDs use:

`<LETTER><2-digit number>`

Letters map to semantic domains:

- **A** = Action semantics (core goal state invariants)
- **L** = Lifecycle semantics (core lifecycle invariants)
- **P** = Parameter semantics (core parameter invariants)
- **G** = Gate semantics (support semantics tracked separately)
- **S** = System semantics (composition/executor/ecosystem/system_contract)

Examples:
- A01, A02, …, A14
- L01, …, L10
- P01, …, P14
- G01
- S01, …, S19

Rules:
- Each Spec ID corresponds to **one atomic invariant** that is testable.
- Spec IDs must be stable: do not renumber unless the spec itself is restructured.

---

## 2) Scenario ID scheme

Scenario IDs use:

`<SpecID>_<short_snake_case_summary>`

Examples:
- `A01_no_ghosts_after_rejection`
- `A06_status_sequence_monotonic`
- `L05_concurrent_transition_rejected`
- `P04_no_partial_application`
- `G01_inactive_suppresses_pubs`

Rules:
- Scenario IDs are stable: do not rename lightly (they appear in traces and audit logs).
- The summary portion should be short, descriptive, and free of versioning words (“final”, “v2”, “new”).

---

## 3) Cardinality rule (ownership)

Default rule:

**One Spec ID → one primary oracle scenario**

Notes:
- A scenario may incidentally validate other invariants, but it has exactly one owning Spec ID.
- If an invariant genuinely requires multiple independent scenarios, then:
  - keep the same Spec ID
  - create additional scenarios with suffixes:
    - `<SpecID>_<summary>_case1`
    - `<SpecID>_<summary>_case2`
  - and list all in traceability.md

Avoid:
- mega-scenarios spanning many Spec IDs
- scenarios named by implementation details instead of semantic outcomes

---

## 4) Layer → oracle coverage rules

The `Layer` column in `docs/workbench/traceability.md` drives where the invariant belongs:

- **Core** invariants:
  - candidates for ORACLE_A enforcement
  - also candidates for ORACLE_B confirmation
- **Policy** invariants:
  - typically ORACLE_B only
  - ORACLE_A may model decision envelopes but must not encode policy meaning
- **Tooling** invariants:
  - often “observe-only” or ORACLE_B
  - enforce only if mechanically testable without relying on tool behaviour
- **System** invariants:
  - ORACLE_B and/or “observe-only”
  - may require manual audit or external observability

Principle:
- ORACLE_A aims to be minimal and mechanistic.
- ORACLE_B is allowed to include orchestration and policy layers (Nav2).

---

## 5) Trace requirements

Every oracle run must emit a `run_start` event including:
- `backend`: `stub` or `ros`
- if `backend=ros`:
  - `ros_distro` from `ROS_DISTRO`
  - `rmw_implementation` from `RMW_IMPLEMENTATION` (or `default` if unset)

Every scenario must emit:
- `scenario_start`
- `scenario_end` with `{ ok: true/false }` in detail

Assertions must emit:
- `assertion` events with:
  - `ok`
  - `expected`
  - `observed`

The audit tool (`audit_trace`) computes verdicts only from the trace.

---

## 6) Traceability requirements

For every Spec ID row in `docs/workbench/traceability.md`:
- `Oracle scenario(s)` must list the scenario ID(s) that claim coverage
- ORACLE_A / ORACLE_B status must be one of:
  - Planned
  - Implemented
  - Verified

Definitions:
- Planned: scenario not yet present or not runnable
- Implemented: scenario runnable and produces a trace
- Verified: scenario is wired to the intended backend and is considered evidence-bearing
  - (stub-only counts as Implemented, not Verified, for ROS semantics)

---

## 7) File naming / location conventions

- Scenario definitions live in:
  - `harness/configs/scenarios*.json`
- Trace output is JSONL:
  - e.g. `/tmp/ros2_semantic_trace.jsonl`
- Oracle runners:
  - `oracle_a` (stub)
  - `oracle_a_ros` (ROS)
- Auditor:
  - `audit_trace`

---

## 8) When to add new scenarios

Add new scenarios only when:
- the owning Spec ID exists in `traceability.md`
- the scenario name follows the rules above
- the scenario produces trace events + audit verdicts
