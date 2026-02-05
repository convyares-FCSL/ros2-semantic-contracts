# Action Scenario Semantics (A)

This document defines the **semantic claims** made by Action oracle scenarios.

It answers the question:
> **What does scenario A (actions) assert about the observable world?**

This document does **not** track implementation status, backend coverage, or production adoption.

Each entry defines:
* the **semantic invariant** being exercised
* what **must be observable**
* what **must not** be inferred (Non-claims)

---

## **Conventions**
* **Normative**: derived from top-level ROS 2 specifications or core semantic contracts.
* **Baseline hypothesis**: expected behaviour inferred from implementation/ecosystem, not yet validated.
* **Policy-layer**: behaviour dependent on higher-level orchestration (e.g. Nav2).

---

### A01 — No Ghosts After Rejection
**Validates:** `SPEC_AC01` (Core)
**Layer:** Core

**Claim**
If a goal is rejected, it MUST NOT become observable as an active or completed goal.

> **Note:** Goal ID uniqueness is infrastructure-provided (UUID generation). This scenario validates only the "no ghosts" aspect: rejected goals must not appear in status/feedback/result streams.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Stimulus: Generic actor configured with `goal_response='reject'` rejects the goal.
- A rejected goal ID never appears in status, feedback, or result streams.
- No terminal or intermediate state is emitted for the rejected goal.

**Non-claims**
- Does not assert how rejection is implemented internally.
- Does not assert specific timing guarantees for the rejection response.
</details>

### A02 — Terminal State Immutability
**Validates:** `SPEC_AC03` (Core)
**Layer:** Core

**Claim**
Once a goal reaches a terminal state (`Succeeded`, `Aborted`, `Canceled`), its status is immutable.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Stimulus: Generic actor configured with `outcome_sequence='succeed,abort'` accepts, succeeds, then attempts abort.
- Any subsequent attempt to change terminal state is rejected.
- No conflicting terminal events are emitted.

**Non-claims**
- Does not assert that the goal memory is instantly freed (retention applies).
</details>

### A03 — Unknown Goal Not Persistent
**Validates:** `SPEC_AC01` (Core)
**Layer:** Core

**Claim**
Goals that are unknown to the system MUST NOT persist as observable entities.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- An unknown goal ID is never observable in status arrays.
- It is treated as if it was never created.

**Status**
- Scenario defined but not yet fully judged in harness.
</details>

### A04 — Valid Transitions Only
**Validates:** `SPEC_AC04` (Core)
**Layer:** Core

**Claim**
A goal can only transition between states defined in the canonical goal state machine.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Transitions follow the directed graph (e.g. `Accepted` -> `Executing`).
- Teleporting across the graph is forbidden.

**Non-claims**
- Does not assert whether transitions are event-based or time-driven.
</details>

### A05 — Invalid Transitions Rejected
**Validates:** `SPEC_AC04` (Core)
**Layer:** Core

**Claim**
Invalid transitions are rejected deterministically by the system.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Any transition not in the valid state machine results in no state change.
- Rejection does not crash the server.
</details>

### A06 — Status Updates Monotonic
**Validates:** `SPEC_AC05` (Core)
**Layer:** Core

**Claim**
Goal status updates are monotonic per goal.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Status can only advance or remain the same.
- Status never regresses (e.g. `Executing` -> `Accepted` is forbidden).
</details>

### A07 — Ordering by Sequence
**Validates:** `SPEC_AC05` (Core)
**Layer:** Core

**Claim**
Events associated with a goal are ordered deterministically by sequence ID, independent of wall-clock time.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Reordering messages by timestamp must not violate the logical sequence.
</details>

### A08 — Cancellation Intent Observable
**Validates:** `SPEC_AC06` (Core)
**Layer:** Core

**Claim**
The intent to cancel a goal must be observable via the cancel service and status topic.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- A "cancel goal" request results in a transition to `Canceling`.
- The cancellation intent is explicitly emitted.
</details>

### A09 — Cancel Resolution Policy
**Validates:** `SPEC_AC06` (Core)
**Layer:** Core

**Claim**
The outcome of a cancellation request is policy-defined (can be `Canceled`, `Aborted`, or `Succeeded`).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The system must apply a policy to decide the result.
- The core allows any terminal resolution from `Canceling`.
</details>

### A10 — Result Visible After Terminal
**Validates:** `SPEC_AC07` (Core)
**Layer:** Core

**Claim**
The result of a goal is observable after it reaches a terminal state.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `GetResult` returns successfully once the goal is terminal.
- The result persists for the retention window.
</details>

### A11 — Single Terminal Result
**Validates:** `SPEC_AC07` (Core)
**Layer:** Core

**Claim**
A goal will only have one terminal result.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Once a goal is terminal, no second result is produced.
- Multiple terminal events for the same goal are impossible.
</details>

### A12 — Supersession via CANCELED
**Validates:** `SPEC_A03` (Global)
**Layer:** Policy

**Claim**
When a goal is superseded (preempted), the previous goal MUST transition to `CANCELED`.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Supersession does NOT result in `ABORTED`.
- The state transition clearly indicates preemption.

**Non-claims**
- Does not specify how or when the supersession event occurs relative to the new goal acceptance (race conditions may exist).
</details>

### A13 — Result Retention Non-Zero
**Validates:** `SPEC_A05` (Global)
**Layer:** Policy

**Claim**
The result of a goal must be retained for observation after the goal terminates.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `GetResult` succeeds for a bounded time after termination.
- The result should not be overwritten by subsequent state changes.

**Non-claims**
- Does not specify exact retention duration (implementation defined).
</details>

### A14 — Tooling Compatibility
**Validates:** `SPEC_A07` (Global)
**Layer:** Tooling

**Claim**
The action server is compatible with standard CLI tools (`ros2 action`).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `list`, `info`, and `send_goal` CLI commands function correctly.

**Non-claims**
- Does not guarantee compatibility with every possible third-party tool, only standard ROS 2 CLI.
</details>

### A15 — No Goal ID Reuse
**Validates:** `SPEC_AC02` (Core)
**Layer:** Core
**Status:** ⚠️ Deferred — requires adversarial UUID-reuse stimulus not expressible in the current ops model; ops are not yet populated.

**Claim**
Goal IDs are never reused, even after the original goal is destroyed.

> **Note:** UUID non-collision is infrastructure-provided. This scenario validates that the action server does not accept a new goal with a previously-used ID within the same session.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- A new goal with a previously used UUID is rejected or treated as a separate entity (if the previous is fully forgotten), but concurrent reuse is forbidden.
</details>

### A17 — Abandoned Goal Cleanup
**Validates:** `SPEC_SYS06` (System)
**Layer:** System

**Claim**
Goals from vanished clients are eventually cleaned up.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Goals do not remain in `Accepted` or `Executing` indefinitely if the client disconnects.
</details>
### A16 — Action Interface Conformity
**Validates:** `SPEC_A01` (Global)
**Layer:** Global
**Claim:** The node exposes the standard ROS 2 action interface graph topology.
<details>
<summary>Assertions</summary>
- Services `_action/send_goal`, `_action/cancel_goal`, `_action/get_result` exist.
- Topics `_action/status`, `_action/feedback` exist.
</details>
