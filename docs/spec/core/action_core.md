# Actions — Core Semantic Contract

Normative status note:
This document may contain sections marked **UNVALIDATED (baseline hypothesis)**.
Such sections describe expected baseline behaviour that is not yet enforced by oracle tests.
They are not part of the normative contract until validated.

---

This document defines the **normative, transport-agnostic semantic contract** for ROS 2
actions as enforced by a core semantics engine.

It specifies goal lifecycle, state transitions, and ordering invariants independently
of ROS services, executors, threading, or QoS.

Canonical global specification:
- `docs/spec/action.md`

---

## 1) Scope and non-goals

This contract applies to:
- goal identity and lifetime
- state transitions and terminality
- cancellation semantics
- ordering and visibility invariants

This contract explicitly excludes:
- ROS action services and message types
- executor behaviour and threading
- lifecycle gating or orchestration
- supersession policy
- QoS configuration

---

## 2) Goal identity

- Each goal is uniquely identified.
- Identity MUST remain stable for the lifetime of the goal.
- Identity reuse after terminal resolution is forbidden.

---

## 3) Goal states

Non-terminal states:
- Accepted
- Executing
- Canceling
- Unknown

Terminal states:
- Succeeded
- Aborted
- Canceled

Rules:
- Terminal states are immutable.
- A goal in a terminal state MUST NOT transition further.
- `Unknown` is not a terminal state and MUST NOT be persisted for a known goal.

---

## 4) State transition validity

Permitted transitions include:
- Accepted → Executing
- Accepted → Succeeded
- Accepted → Aborted
- Accepted → Canceling
- Accepted → Canceled
- Executing → Succeeded
- Executing → Aborted
- Executing → Canceling
- Canceling → Canceled

Invalid transitions MUST be rejected deterministically.

---

## 5) Ordering and visibility

- Status updates MUST be monotonically ordered per goal
  (strictly increasing sequence).
- Ordering MUST NOT depend on wall-clock time.
- Identical transition sequences MUST produce identical observable orderings.

---

## 6) Cancellation semantics

- Cancellation intent is explicit and observable.
- Cancellation does not guarantee a specific terminal outcome.

While in `Canceling`, resolution to any terminal state permitted by the active
cancellation policy is allowed.

The core validates that any terminal outcome is permitted by policy;
it does **not** select the outcome.

---

## 7) Result visibility

- A result is visible only after terminal resolution.
- A goal MUST have at most one terminal result.
- Result visibility MUST be consistent with goal state.

---

## 8) Invariants (testable)

The core MUST enforce:
- Unique goal identity
- Valid state transitions only
- Terminal state immutability
- Deterministic ordering
- Policy-constrained terminal resolution
- No persistence of `Unknown` for known goals

---

## Provenance

### Upstream sources
- [Design Article: Actions](https://design.ros2.org/articles/actions.html)
- [ROS 2 Docs: About Actions](https://docs.ros.org/en/jazzy/Concepts/Basic-Concepts/About-Actions.html)
- [ROS 2 Docs: Writing an Action Server (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [ROS 2 Docs: Understanding Actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

### Implementation-defined (rclcpp)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Ecosystem-defined (Nav2)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Project policy
- None
