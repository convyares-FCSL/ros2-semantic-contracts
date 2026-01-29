# Lifecycle — Global Semantic Specification

Normative status note:
This document may contain sections marked **UNVALIDATED (baseline hypothesis)**.
Such sections describe expected baseline behaviour that is not yet enforced by oracle tests.
They are not part of the normative contract until validated.

---

This document defines the **ROS-facing semantic contract** for managed lifecycle nodes.

It specifies externally observable lifecycle behaviour as seen by clients, tools,
and orchestration frameworks.

Core semantic invariants are defined in:
- `docs/spec/core/lifecycle_core.md`

---

## 1) Scope

This specification applies to:
- ROS lifecycle services
- lifecycle state introspection
- externally observable transition outcomes

It does not define:
- internal implementation mechanisms
- executor behaviour
- scheduling or timing guarantees
- ecosystem orchestration policies

---

## 2) Interfaces

Lifecycle semantics are exposed via standard ROS 2 lifecycle services and messages.

Exact service names and message types are defined by ROS 2.
This specification does not redefine the IDL.

---

## 3) Lifecycle state visibility

- `get_state` MUST report only primary lifecycle states.
- Transition states MUST NOT be reported as stable states.
- State queries MUST reflect the resolved lifecycle state.

---

## 4) Transition requests

### Acceptance and rejection

- Valid transition requests MUST be accepted or rejected deterministically.
- Invalid or busy requests MUST be rejected.
- Rejection MUST be observable via the service response.

Whether a busy or invalid rejection emits a `TransitionEvent` is
implementation-defined and enforced by the baseline profile,
but MUST be consistent and documented.

---

## 5) Transition completion and events

> ⚠️ **UNVALIDATED (baseline hypothesis)**
>
> Baseline expectation:
> - Accepted transition attempts emit a `TransitionEvent`.
> - One event is emitted per accepted transition attempt.
>
> Observers MUST NOT assume event emission semantics are identical
> across different lifecycle implementations unless validated.

---

## 6) Shutdown semantics

- Shutdown requests MUST resolve deterministically.
- Resolution MUST result in Finalized or a documented failure outcome.
- Nodes MUST NOT remain indefinitely in a transition state.

Deterministic resolution includes bounded completion or
a documented failure outcome.

---

## 7) Lifecycle-managed publishers

> ⚠️ **UNVALIDATED (baseline hypothesis)**
>
> Baseline expectation (Jazzy+rclcpp):
> - While Inactive, publication via lifecycle-managed publishers is suppressed.

While Inactive, a lifecycle-managed node MUST NOT publish messages
via lifecycle-managed publishers.

This rule applies only to lifecycle-managed publishers.
No claims are made regarding timers, subscriptions, or actions.

---

## 8) Tooling compatibility

A compliant implementation MUST support lifecycle introspection
via standard ROS 2 tools.

Tooling compatibility is validated via oracle and harness testing,
not by assertion alone.

---

## Provenance

### Upstream sources
- [Design Article: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 Docs: Lifecycle Package](https://docs.ros.org/en/jazzy/p/lifecycle/)

### Implementation-defined (rclcpp)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Ecosystem-defined (Nav2)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Project policy
- Stable, observable lifecycle semantics
