# Parameters — Global Semantic Specification

**Normativity Class: Global Specification**

This document defines the **ROS-facing semantic contract** for parameters: what clients, tools, and observers can rely on when interacting with a node’s parameters.

It defines observable meaning, not implementation strategy.
Core semantic invariants are defined in `docs/spec/core/parameter_core.md`.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- ROS parameter services
- parameter introspection and tooling
- externally observable parameter behaviour
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- internal storage mechanisms
- executor or threading behaviour
- lifecycle gating policies
</details>

---

## Interfaces

### SPEC_P01 — Standard Parameter Services [P15]

Parameter semantics MUST be exposed via standard ROS 2 parameter services and messages.
- Exact service names, message types, and IDL are defined by ROS 2.
- This specification does not redefine the IDL.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

</details>

---

## Declaration modes

### SPEC_P02 — Explicit Declaration Mode [P16]

Nodes MAY operate in **declared-only mode** or **undeclared-allowed mode**.
- The chosen mode MUST be documented by the implementation.
- **Baseline Requirement:** If `allow_undeclared_parameters` is false (default), setting an undeclared parameter MUST fail.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp default behavior

</details>

---

## GetParameters behaviour

### SPEC_P03 — Unknown Parameters Introspection Behavior [P08]

In declared-only mode:
- `get_parameters` on unknown or undeclared names MUST return `NOT_SET`-typed values for requested keys.
- Implementations MUST document their `get_parameter_types` behavior for undeclared names.
- `get_parameter_types` 1:1 response conformance (including undeclared-name mapping) is governed by `SPEC_PC12`, not redefined here.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

**Notes**
- Baseline profile observed in this repository for mixed declared/undeclared `get_parameter_types` requests is `successful=false`, `reason=NotDeclared`, `count=0`, `types=[]` (`P24` evidence path).
- That observed profile is tracked as divergence evidence and MUST NOT replace the `SPEC_PC12` conformance requirement.

</details>

---

## SetParameters behaviour

### SPEC_P04 — Atomic-Batch Parameter Updates [P04, P09, P10, P18]

`set_parameters_atomically` requests MUST be atomic per request.
- Partial application MUST NOT be externally observable for atomic-batch requests.
- Rejected atomic-batch updates MUST NOT emit parameter change events.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

</details>

### SPEC_P09 — Non-Atomic Batch and Observer Visibility [P17, P25]

`set_parameters` (non-atomic batch) is a distinct API profile from `set_parameters_atomically`.
- Partial application MAY be externally observable when elements in a batch have mixed accept/reject outcomes.
- Under concurrent reads, mixed old/new key visibility is probe-dependent and must be established by issue-equivalent evidence.
- Clients MUST NOT assume cross-key snapshot consistency for non-atomic batch operations.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Baseline: Jazzy+rclcpp and Jazzy+rclpy (`P17`, `P25`)

**Notes**
- This clause is intentionally scoped to observable wire/API behavior, not scheduler internals.
- Current `P25` path is a repeated single-call probe and observed mixed snapshots on the measured Jazzy path.

</details>

### SPEC_P10 — Atomicity Profiles and Commit-Gated Visibility [P27, P28, P29, P30, P33, P34, P41]

⚠️ **UNVALIDATED (baseline hypothesis)**

Clients and observers MUST treat standard and atomic set APIs as distinct observable profiles:
- `set_parameters` (standard): sequential profile; mixed accept/reject outcomes may leave earlier successful keys committed.
- `set_parameters_atomically` (atomic): transactional profile; any failing element rejects the whole request.

Visibility contract:
- Rejected updates MUST remain externally silent (no committed-value visibility).
- Successful updates become externally visible only after validation success and commit.
- Callback-time storage reads MUST reflect old committed values until commit.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp and Jazzy+rclpy measured behavior (`P17`, `P18`, `P27`-`P30`, `P33`, `P34`, `P41`)

**Notes**
- This clause captures observer-facing timing semantics; it does not constrain internal locking implementation.
- Pinned Docker Jazzy+Humble evidence exists for this scenario family across `prod` and `py`.
- `P41` adds a counterfactual control that rejects pre-commit visibility assumptions and passed on current `prod`/`py` Jazzy runs.

</details>

---

## Parameter events

### SPEC_P05 — Commit-Only Event Semantics [P10, P11]

Parameter events describe committed state changes, not rejected attempts.
- Successful committed updates MUST be observable via change payloads that match committed value/type state.
- Rejected updates MUST NOT appear as committed change events.

Retired in `D-P2-01`:
- Startup/non-atomic event cardinality and full deterministic envelope claims were removed from closure (`P20`, `P32` remain probe-only signature scenarios).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp, Jazzy+rclpy (`P10`, `P11`)

**Notes**
- This clause is intentionally bounded to commit-vs-reject observables.

</details>

---

## Introspection

### SPEC_P06 — Unknown Describe Profile Stability [P12]

For repeated `describe_parameters` calls on the same unknown key, observable outcome shape MUST be stable within the tested process profile.
- The returned descriptor default fields MUST remain stable across repeated calls.
- The success/failure shape for unknown keys MUST be consistent for the implementation profile.
- Unknown-key describe operations MUST NOT mutate unrelated declared parameter state.
- Error reason text is implementation-defined and not normative.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp

</details>

### SPEC_P07 — Stable List Enumeration [P13]

⚠️ **UNVALIDATED (baseline hypothesis)**

For `list_parameters`:
- Returned names MUST be complete (subject to depth/prefix filters).
- Returned names MUST be stable across identical queries.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp

</details>

---

## Deletion semantics

### SPEC_P08 — Optional Deletion Profile [P14, P42]

Parameter deletion support is optional.
- If supported, deleting a declared key MUST remove it from parameter read/list surfaces.
- Deleting an unknown key MUST fail or no-op without mutating unrelated declared keys.
- Emission of a delete `parameter_event` is implementation-profile behavior and MUST be explicitly documented.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp, Jazzy+rclpy (`P14`, `P42`)

**Notes**
- Current measured Jazzy profile in this repository: successful undeclare removes state and emits no delete event (`P14`).
- Unknown undeclare returns `NotDeclared` and preserves unrelated state (`P42`).

</details>
