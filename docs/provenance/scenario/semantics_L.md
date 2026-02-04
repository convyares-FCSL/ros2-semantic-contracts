# Lifecycle Scenario Semantics (L)

This document defines the **semantic claims** made by Lifecycle oracle scenarios.

It answers the question:
> **What does scenario L (lifecycle) assert about the observable world?**

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

### L01 — Only Defined Transitions
**Validates:** `SPEC_LC02` (Core)
**Layer:** Core

**Claim**
Only lifecycle transitions defined by the canonical state machine are permitted.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Teleporting from `Unconfigured` to `Active` is rejected.
- Only valid edges in the state graph are traversable.
</details>

### L02 — Primary State Visibility
**Validates:** `SPEC_LC01` (Core)
**Layer:** Core

**Claim**
The lifecycle node exposes only primary states externally; auxiliary states are internal.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Primary states (`Unconfigured`, `Inactive`, `Active`, `Finalized`) are queryable.
- Auxiliary states (`Configuring`, `Activating`, etc.) are not reported as stable states.

**Scope note**
- L01 validates transition graph validity; L02 validates state visibility.
</details>

### L03 — Intermediate States Hidden
**Validates:** `SPEC_LC03` (Core)
**Layer:** Core

**Claim**
Intermediate lifecycle states (`Configuring`, `Activating`) are not observable as stable states via `GetState`.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `GetState` never returns an auxiliary state.
- Transition events denote entry/exit of these states, but they are transient.
</details>

### L04 — Single Active Transition (Lock Aspect)
**Validates:** `SPEC_LC04` (Core)
**Layer:** Core

**Claim**
Only one transition can be active at any given time.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The engine locks the state machine during a transition.
- A second transition request during an active transition is not processed.

**Scope note**
- L04 validates the lock invariant; L05 validates that rejection is observable.
</details>

### L05 — Concurrent Transition Rejected (Observable Rejection)
**Validates:** `SPEC_LC04` (Core)
**Layer:** Core

**Claim**
Concurrent transition requests are rejected with observable failure (Busy).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- If `change_state` is called while another transition is running, it returns failure/busy.
- The rejection is deterministic and observable via service response.

**Scope note**
- L04 validates the lock invariant; L05 validates that rejection is observable.
</details>

### L06 — ErrorProcessing Deterministic
**Validates:** `SPEC_LC05` (Core)
**Layer:** Core

**Claim**
Error processing in the lifecycle resolves deterministically to a primary state.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `on_error` returning SUCCESS resolves to `Unconfigured`.
- `on_error` returning FAILURE resolves to `Finalized`.
</details>

### L07 — GetState Reports Primary Only
**Validates:** `SPEC_L02` (Global)
**Layer:** Core

**Claim**
The `GetState` service MUST only return primary states (`Unconfigured`, `Inactive`, `Active`, `Finalized`).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Responses are filtered to exclude internal transition states.
</details>

### L08 — Rejection Observable
**Validates:** `SPEC_L03` (Global)
**Layer:** Core

**Claim**
Rejection of a lifecycle transition must be observable via the service response.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The `ChangeState` service returns a non-success boolean/code.
- Rejection must be clear, including the reason (if supported).
</details>

### L09 — TransitionEvent Emitted
**Validates:** `SPEC_L04` (Global)
**Layer:** Core

**Claim**
A `TransitionEvent` is emitted when a transition occurs.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The `/transition_event` topic receives a message describing the start and goal states.
- Events are triggered for every transition (whether successful or rejected).
</details>

### L10 — Shutdown Deterministic
**Validates:** `SPEC_L05` (Global)
**Layer:** Core

**Claim**
Shutdown of the node is deterministic and results in `Finalized`.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The node ends in the `Finalized` state.
- Resources are cleaned up.
</details>

### L11 — Shutdown via Deactivate
**Validates:** `SPEC_L06` (Global)
**Layer:** Core

**Claim**
Shutdown from `Active` implies a logical deactivation.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `on_deactivate` logic runs before `on_shutdown`.
</details>

### L12 — Services Not Gated
**Validates:** `SPEC_L08` (Global)
**Layer:** Core

**Claim**
Custom user services are available in `Unconfigured` and `Inactive` states (unless manually gated).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Service calls succeed even when the node is not `Active`.
</details>
### L13 — Lifecycle Interface Conformity
**Validates:** `SPEC_L01` (Global)
**Layer:** Global
**Claim:** The node exposes the standard ROS 2 lifecycle interface graph topology.
<details>
<summary>Assertions</summary>
- Services `change_state`, `get_state`, `get_available_states`, `get_available_transitions` exist.
- Topic `transition_event` exists.
</details>
