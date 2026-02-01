# Actions — Core Semantic Contract

**Normativity Class: Core Contract**

This document defines the **normative, transport-agnostic semantic contract** for ROS 2 actions as enforced by a core semantics engine.
It specifies goal lifecycle, state transitions, and ordering invariants independently of ROS services, executors, threading, or QoS.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- goal identity and lifetime
- state transitions and terminality
- cancellation semantics (engine level)
- ordering and visibility invariants
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- ROS action services and wire protocol (see `docs/spec/global/action.md`)
- executor behaviour and threading
- lifecycle gating or orchestration
- supersession policy (system level)
- QoS configuration
</details>

---

## Goal Identity

### SPEC_AC01 — Unique Goal Identity [S01]

Each goal processed by the engine MUST be uniquely identified.
- Two distinct goals MUST NOT share the same identifier (UUID) concurrently.
- The engine MUST treat goals with distinct IDs as semantically independent entities.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

### SPEC_AC02 — Identity Stability and No Reuse [A16]

Goal identity MUST be immutable and non-reusable.
- Identity MUST remain stable for the entire lifetime of the goal.
- **Identity Reuse Forbidden:** Once a goal ID reaches a terminal state, that ID MUST NOT be reused for a new goal within the server's execution context.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html) (Implies uniqueness over time)
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Safety requirement)

**Notes**
- Reusing IDs destroys the ability for clients to correlate late results or status updates.

</details>

---

## Goal States and Terminality

### SPEC_AC03 — Terminal State Immutability [S01]

Goals exist in **Non-terminal** (`Accepted`, `Executing`, `Canceling`) or **Terminal** (`Succeeded`, `Aborted`, `Canceled`) states.
- Terminal states are **immutable**.
- A goal in a terminal state MUST NOT transition to any other state.
- `Unknown` is NOT a terminal state; it indicates absence of state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)

**Notes**
- The "Unknown" state exists for external observers but is not a valid internal state for an active goal.

</details>

---

## State Transition Validity

### SPEC_AC04 — Valid Transition Graph [S01]

The core engine MUST enforce a strict state transition graph.
- Invalid transitions MUST be rejected deterministically.

**Permitted Transitions:**
1.  **From Accepted:**
    * → `Executing` (Normal start)
    * → `Canceling` (Cancel requested before execution)
    * → `Succeeded` / `Aborted` / `Canceled` (Fast termination)
2.  **From Executing:**
    * → `Canceling` (Cancel requested during execution)
    * → `Succeeded` / `Aborted` (Normal termination)
3.  **From Canceling:**
    * → `Canceled` (Cancel accepted)
    * → `Succeeded` / `Aborted` (Cancel ignored/race condition)

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)

**Notes**
- Transitioning from `Canceling` to `Succeeded` is valid "Physics" (the core allows it), even if specific policies might discourage it.

</details>

---

## Ordering Invariants

### SPEC_AC05 — Monotonic Status Ordering [S01]

Status updates exposed by the core MUST be monotonically ordered per goal.
- The sequence of states observed for a single goal MUST strictly follow the transition graph.
- Ordering MUST NOT depend on wall-clock time.
- Identical transition sequences MUST produce identical observable orderings.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

---

## Cancellation Semantics

### SPEC_AC06 — Cancellation Intent and Resolution [S01]

Cancellation is a request, not a coerced outcome.
- Cancellation intent MUST be explicit and observable (transition to `Canceling`).
- Transitioning to `Canceling` does **not** guarantee a specific terminal outcome.
- The core MUST allow resolution to *any* terminal state (`Succeeded`, `Aborted`, or `Canceled`) from the `Canceling` state, subject to policy.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)

**Notes**
- The core validates that the transition is *possible*; the user code (Policy) decides which outcome occurs.

</details>

---

## Result Visibility

### SPEC_AC07 — Result Singularity and Consistency [S01]

- A goal MUST have at most one terminal result.
- A result is semantically valid only after terminal resolution.
- Result visibility MUST be consistent with the goal state (e.g., a `Succeeded` goal cannot yield a "Canceled" result payload, though the payload content itself is user-defined).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)

</details>