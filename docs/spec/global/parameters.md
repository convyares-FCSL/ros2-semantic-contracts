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

### SPEC_P03 — Unknown Parameters Return NOT_SET [P08]

For both `get_parameters` and `get_parameter_types`:
- Unknown or undeclared parameter names MUST return a value of type `NOT_SET`.
- This behaviour is normative and stable.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

**Notes**
- Allows tooling to reason about unknown parameters without ambiguous failures.

</details>

---

## SetParameters behaviour

### SPEC_P04 — Atomic Parameter Updates [P04, P09, P10]

Parameter updates MUST be atomic per request.
- Partial application MUST NOT be externally observable.
- Rejected updates MUST NOT emit parameter change events.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

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

Parameter deletion support is optional.
- If supported, deletion MUST be documented.
- Deleted parameters MUST be reported via parameter events.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- REP/RFC: None
- Community: None

</details>