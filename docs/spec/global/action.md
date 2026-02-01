# Actions — Global Semantic Specification

**Normativity Class: Global Specification**

This document defines the **ROS-facing semantic contract** for actions: what clients, tools, and observers can rely on when interacting with action servers.

It defines observable meaning, not implementation strategy.
Core semantic invariants are defined in `docs/spec/core/action_core.md`.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- action goal submission, cancellation, and result retrieval
- goal status reporting
- externally observable action behaviour
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- internal execution models
- lifecycle gating policies
- executor behaviour
- QoS configuration details
</details>

---

## Interfaces

### SPEC_A01 — Standard Action Services [S01]

Action semantics MUST be exposed via standard ROS 2 action services and topics.
- Exact names and message types are defined by the ROS 2 action IDL.
- This specification does not redefine the IDL.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

---

## Goal lifecycle and outcomes

### SPEC_A02 — Observable Lifecycle and Outcomes [S01]

Each goal MUST proceed through a well-defined lifecycle.
- Terminal outcomes MUST convey semantic meaning to clients.
- Terminal states MUST be externally observable via status topics.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

---

## Supersession semantics

### SPEC_A03 — Supersession is Distinct from Failure [A12]

⚠️ **UNVALIDATED (baseline hypothesis)**

When a new goal supersedes an existing active goal (preemption):
- The previously active goal MUST transition to a terminal state whose semantic meaning clearly indicates supersession.
- Supersession MUST be externally distinguishable from terminal states that indicate internal failure or error.
- **Baseline Requirement:** Supersession MUST be represented as `CANCELED`.
- Using `ABORTED` to represent supersession is non-compliant.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on specific state for preemption)
- REP/RFC: None
- Community: Nav2 (Safety requirement)
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Semantic distinction required for replanning)

</details>

---

## Ordering guarantees

### SPEC_A04 — Per-Goal Ordering [S01]

Status and feedback updates MUST be ordered per goal.
- Ordering is defined by per-goal sequence progression, not wall-clock time.
- Clients MUST NOT infer ordering from timestamps.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

---

## Result retention

### SPEC_A05 — Non-Zero Result Retention [S01]

Results MUST be retained for some non-zero duration to allow late retrieval.
- Retention duration is implementation-defined and MUST be documented.
- Zero-duration retention is non-compliant unless explicitly documented.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

---

## Cancellation behaviour

### SPEC_A06 — Observable Cancellation Intent [S01]

Cancellation intent MUST be observable via the cancel service.
- Cancellation MAY resolve to different terminal outcomes depending on timing and implementation policy.
- No specific terminal outcome is guaranteed by cancellation alone (race conditions exist).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)
- REP/RFC: None
- Community: None

</details>

---

## Tooling compatibility

### SPEC_A07 — Tooling Introspection [S14]

A compliant implementation MUST support standard ROS 2 tools for:
- action discovery
- goal status introspection
- result retrieval

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Tooling observability)

</details>

---

## Identity and Cleanup

### SPEC_A08 — Goal Identity Reuse Forbidden [S01]

Goal identifiers (UUIDs) MUST NOT be reused within the lifetime of the action server.
- Once a goal UUID reaches a terminal state, it MUST NOT be used for a subsequent goal.
- Clients and Tools rely on UUID uniqueness to correlate results.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html) (Implies UUID uniqueness)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Safety against race conditions)

</details>

### SPEC_A09 — Abandoned Goal Cleanup [A17]

⚠️ **UNVALIDATED (baseline hypothesis)**

Action servers MUST eventually clean up goals from clients that have vanished or abandoned the goal.
- Cleanup MUST occur within a bounded observation window (e.g., timeout or session expiry).
- This prevents unbounded resource leaks in long-running servers.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: Nav2 (Operational stability)
- BIC: [system_contract.md#baseline-interoperability-constraints](../system/system_contract.md) (Resource leak prevention)

**Notes**
- "Vanished" implies the client node no longer exists in discovery, or the transport link is severed.

</details>