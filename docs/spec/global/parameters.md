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

### SPEC_P03 — Unknown Parameters Introspection Behavior [P08, P24]

In declared-only mode:
- `get_parameters` on unknown or undeclared names returns `NOT_SET`-typed values.
- `get_parameter_types` with any undeclared name fails with `NotDeclared` in the Jazzy baseline profile.
- Callers MUST NOT assume `get_parameter_types` returns a 1:1 `NOT_SET` mapping for mixed declared/undeclared requests.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

**Notes**
- This clause captures observed Jazzy+rclcpp behavior and matching backend_py behavior in this repository.

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

### SPEC_P10 — Atomicity Profiles and Commit-Gated Visibility [P27, P28, P29, P30, P33, P34]

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
- Baseline: Jazzy+rclcpp and Jazzy+rclpy measured behavior (`P17`, `P18`, `P27`-`P30`, `P33`, `P34`)

**Notes**
- This clause captures observer-facing timing semantics; it does not constrain internal locking implementation.
- Pinned Docker Jazzy+Humble evidence exists for this scenario family across `prod` and `py`.

</details>

---

## Parameter events

### SPEC_P05 — Deterministic Event Emission [P07, P11]

⚠️ **UNVALIDATED (baseline hypothesis)**

Parameter change events MUST describe **state changes**, not attempts.
- Observers MUST NOT rely on event count as a proxy for operation count.
- Ordering and content MUST remain deterministic.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp observed behavior

</details>

---

## Introspection

### SPEC_P06 — Consistent Description [P12]

Behaviour for `describe_parameters` on unknown parameters MUST be consistent.
- If descriptors are returned for unknown parameters, their semantics MUST be documented.
- No default descriptor behaviour is assumed.

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

### SPEC_P08 — Optional Deletion Support [P14]

⚠️ **UNVALIDATED (baseline contradicts)**

Parameter deletion support is optional.
- If supported, deletion MUST be documented.
- Deleted parameters SHOULD be reported via parameter events.
  Baseline observation (Jazzy+rclcpp): `undeclare_parameter` does NOT emit a `ParameterEvent`
  to `/parameter_events`. Top-level docs are silent on deletion event requirements.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

**Notes**
- Changed from MUST to SHOULD: rclcpp baseline does not emit ParameterEvent on undeclare.
  Per authority ladder, rclcpp behavior defines truth when docs are silent.

</details>
