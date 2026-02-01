# Lifecycle — Global Semantic Specification

**Normativity Class: Global Specification**

This document defines the **ROS-facing semantic contract** for managed lifecycle nodes.
It specifies externally observable lifecycle behaviour as seen by clients, tools, and orchestration frameworks.

It defines observable meaning, not implementation strategy.
Core semantic invariants are defined in `docs/spec/core/lifecycle_core.md`.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- ROS lifecycle services
- lifecycle state introspection
- externally observable transition outcomes
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- internal implementation mechanisms
- executor behaviour
- scheduling or timing guarantees
- ecosystem orchestration policies
</details>

---

## Interfaces

### SPEC_L01 — Standard Lifecycle Services [S12]

Lifecycle semantics MUST be exposed via standard ROS 2 lifecycle services and messages.
- Exact service names and message types are defined by ROS 2.
- This specification does not redefine the IDL.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- REP/RFC: None
- Community: None

</details>

---

## Lifecycle state visibility

### SPEC_L02 — Primary State Visibility [S12]

State queries MUST reflect the resolved lifecycle state.
- `get_state` MUST report only primary lifecycle states (`Unconfigured`, `Inactive`, `Active`, `Finalized`).
- Transition states MUST NOT be reported as stable states via `get_state`.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- REP/RFC: None
- Community: None

</details>

---

## Transition requests

### SPEC_L03 — Deterministic Transition Acceptance [S12]

Valid transition requests MUST be accepted or rejected deterministically.
- Invalid or busy requests MUST be rejected.
- Rejection MUST be observable via the service response.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- REP/RFC: None
- Community: None

**Notes**
- Whether a busy/invalid rejection emits a `TransitionEvent` is implementation-defined but MUST be documented.

</details>

---

## Transition completion and events

### SPEC_L04 — Transition Event Emission [S12]

⚠️ **UNVALIDATED (baseline hypothesis)**

Accepted transition attempts MUST emit a `TransitionEvent`.
- One event is emitted per accepted transition attempt.
- Observers MUST NOT assume event emission semantics are identical across different lifecycle implementations unless validated.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp observed behavior

</details>

---

## Shutdown semantics

### SPEC_L05 — Deterministic Shutdown [S12]

Shutdown requests MUST resolve deterministically.
- Resolution MUST result in `Finalized` or a documented failure outcome.
- Nodes MUST NOT remain indefinitely in a transition state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- REP/RFC: None
- Community: None

</details>

### SPEC_L06 — Shutdown via Deactivate (Active State) [L11]

If a shutdown is requested while the node is in the `Active` state:
- The node MUST logically perform deactivation logic before shutdown logic.
- Externally, this MUST manifest as `on_deactivate` transition events (or equivalent effects) before `on_shutdown`.
- Bypassing deactivation logic during active-shutdown is non-compliant.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html) (Implies orderly teardown)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Safety requirement)

</details>

---

## Lifecycle-managed gating

### SPEC_L07 — Publisher Suppression in Inactive State [S12]

While `Inactive`, a lifecycle-managed node MUST NOT publish messages via lifecycle-managed publishers.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- REP/RFC: None
- Community: None

**Notes**
- This rule applies only to lifecycle-managed publishers.
- No claims are made regarding timers, subscriptions, or actions in this spec.

</details>

### SPEC_L08 — Service Gating Negative Assertion [L12]

Custom user services are NOT automatically gated by the `Inactive` state.
- Unless explicitly implemented by the user, services remain available in `Unconfigured` and `Inactive` states.
- Clients MUST NOT assume service unavailability based on lifecycle state alone.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- Baseline: Jazzy+rclcpp observed behavior (Services are not managed by default)

</details>

---

## Tooling compatibility

### SPEC_L09 — Tooling Introspection [S14]

A compliant implementation MUST support lifecycle introspection via standard ROS 2 tools.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Tooling observability)

</details>