# Actions — Global Semantic Specification

Normative status note:
This document may contain sections marked **UNVALIDATED (baseline hypothesis)**.
Such sections describe expected baseline behaviour that is not yet enforced by oracle tests.
They are not part of the normative contract until validated.

---

This document defines the **ROS-facing semantic contract** for actions:
what clients, tools, and observers can rely on when interacting with action servers.

It defines observable meaning, not implementation strategy.

Core semantic invariants are defined in:
- `docs/spec/core/action_core.md`

---

## 1) Scope

This specification applies to:
- action goal submission, cancellation, and result retrieval
- goal status reporting
- externally observable action behaviour

It does not define:
- internal execution models
- lifecycle gating policies
- executor behaviour
- QoS configuration details

---

## 2) Interfaces

Action semantics are exposed via standard ROS 2 action services and topics.

Exact names and message types are defined by the ROS 2 action IDL.
This specification does not redefine the IDL.

---

## 3) Goal lifecycle and outcomes

- Each goal proceeds through a well-defined lifecycle.
- Terminal outcomes convey semantic meaning to clients.
- Terminal states MUST be externally observable.

---

## 4) Supersession semantics

⚠️ **UNVALIDATED (baseline hypothesis)**  
This section describes expected baseline behaviour inferred from ecosystem usage
and implementation inspection. It is not normative until validated by oracle
testing against the Jazzy+rclcpp+Nav2 baseline.

When a new goal supersedes an existing active goal, the previously active goal
MUST transition to a terminal state whose semantic meaning clearly indicates
that it was superseded by a subsequent goal.

Supersession MUST be externally distinguishable from terminal states that
indicate internal failure or error.

In the baseline profile (**Jazzy+rclcpp+Nav2**), supersession is expected to be
represented as `CANCELED`, with result or reason information indicating goal
supersession (preemption).

Using `ABORTED` to represent supersession is non-compliant with the baseline
expectation.

---

## 5) Ordering guarantees

- Status and feedback updates MUST be ordered per goal.
- Ordering is defined by per-goal sequence progression, not wall-clock time.
- Clients MUST NOT infer ordering from timestamps.

---

## 6) Result retention

- Results MUST be retained for some non-zero duration to allow late retrieval.
- Retention duration is implementation-defined and MUST be documented.
- Zero-duration retention is non-compliant unless explicitly documented.

---

## 7) Cancellation behaviour

- Cancellation intent MUST be observable.
- Cancellation MAY resolve to different terminal outcomes depending on timing
  and implementation policy.

No specific terminal outcome is guaranteed by cancellation alone.

---

## 8) Tooling compatibility

A compliant implementation MUST support:
- action discovery
- goal status introspection
- result retrieval via standard ROS 2 tools

Tooling compatibility is validated via oracle and harness testing,
not by assertion alone.

---

## Provenance

### Upstream sources
- [Design Article: Actions](https://design.ros2.org/articles/actions.html)
- [ROS 2 Docs: About Actions](https://docs.ros.org/en/jazzy/Concepts/Basic-Concepts/About-Actions.html)

### Implementation-defined (rclcpp)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Ecosystem-defined (Nav2)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Project policy
- Baseline action semantics for interoperability
