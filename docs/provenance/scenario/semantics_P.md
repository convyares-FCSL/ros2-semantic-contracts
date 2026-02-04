# Parameter Scenario Semantics (P)

This document defines the **semantic claims** made by Parameter oracle scenarios.

It answers the question:
> **What does scenario P (parameters) assert about the observable world?**

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

### P01 — No Fabricated Parameters
**Validates:** `SPEC_PC01` (Core)
**Layer:** Core

**Claim**
Parameters that have not been declared or implicitly allowed MUST NOT appear in `ListParameters`.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Queries for non-existent parameters return `NOT_SET`.
</details>

### P02 — Dynamic Typing Rules
**Validates:** `SPEC_PC03` (Core)
**Layer:** Core

**Claim**
Parameters with `dynamic_typing=true` can change type; otherwise, type is fixed.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- If static, setting a different type fails.
- If dynamic, setting a different type succeeds.

**Non-claims**
- Does not specify how dynamically assigned types are managed within the system.
</details>

### P03 — Redeclaration Fails
**Validates:** `SPEC_PC04` (Core)
**Layer:** Core

**Claim**
Attempting to redeclare an already-declared parameter must fail deterministically.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The `declare_parameter` call throws or returns failure.
- No parameter state change occurs as a result of the failed attempt.

**Non-claims**
- Does not assert the error reason text.
- Does not assert whether the failure is silent or reported via events.

**Important note**
This scenario does **not** assert identity based on parameter name alone. It asserts semantic redeclaration.
</details>

### P04 — No Partial Application
**Validates:** `SPEC_PC05` (Core), `SPEC_P04` (Global)
**Layer:** Core

**Claim**
Atomic parameter updates: either all succeed or all fail.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- If setting Params A and B, and B is invalid, A remains unchanged.
- Any partial application should not affect the system state until fully applied.

**Non-claims**
- Does not specify internal mechanisms for handling partial applications.

**Scope note**
- P04 exercises the *decision* layer: the set-operation outcome is all-or-nothing. P09 exercises the *observation* layer: no partial state is visible to external subscribers. Currently both reduce to the same evidence (state unchanged after rejection); they will diverge once per-subscriber visibility primitives are available.
</details>

### P05 — Read-Only Enforcement
**Validates:** `SPEC_PC05` (Core)
**Layer:** Core

**Claim**
Read-only parameters cannot be modified after declaration.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `SetParameters` returns failure for read-only params.
- The system should provide feedback when modification is attempted.

**Non-claims**
- Does not specify whether the read-only state can be altered programmatically.
</details>

### P06 — Unknown Parameters Rejected
**Validates:** `SPEC_PC06` (Core), `SPEC_P02` (Global)
**Layer:** Core

**Claim**
If `allow_undeclared_parameters` is false, setting an unknown parameter fails.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `SetParameters` returns failure.
- The parameter does not appear in the list.
</details>

### P07 — Change Record Ordering
**Validates:** `SPEC_PC07` (Core), `SPEC_P05` (Global)
**Layer:** Core

**Claim**
The system respects a deterministic record ordering for parameter operations.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Parameter events arrive in the order operations were applied.
- The order should not vary unexpectedly across identical operations.

**Non-claims**
- Does not assert specific implementation details regarding how order is maintained.
</details>

### P08 — Unknown Types NOT_SET
**Validates:** `SPEC_PC03` (Core), `SPEC_P03` (Global)
**Layer:** Core

**Claim**
Unknown parameters are represented with a `NOT_SET` type.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `GetParameters` returns `type: NOT_SET` for missing keys.
- The system should not infer types for unknown parameters.

**Non-claims**
- Does not specify how unknown types are handled internally.
</details>

### P09 — Atomic Updates (External)
**Validates:** `SPEC_P04` (Global)
**Layer:** Core

**Claim**
Parameter updates must be atomic and externally observable.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Observers see the state change in a single event.
- No partial updates should be visible.

**Non-claims**
- Does not define how atomicity is managed at the backend level.

**Scope note**
- Overlaps P04 today (both assert full-reject behaviour). Intended divergence: P09 will exercise external event streams independently when per-subscriber observation primitives are available. See P04 scope note.
</details>

### P10 — Rejected Updates Silent
**Validates:** `SPEC_P04` (Global)
**Layer:** Core

**Claim**
Rejected updates should not emit any parameter change events.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- No `/parameter_events` message is published for a failed set attempt.

**Non-claims**
- Does not specify why or how the rejection occurs.

**Scope note**
- Distinct from P04/P09: P10 asserts *event absence*, not state immutability. A system could pass P04 (state unchanged) while still emitting a spurious event; P10 catches that.
</details>

### P11 — Events Describe Changes
**Validates:** `SPEC_P05` (Global)
**Layer:** Core

**Claim**
Parameter events must describe changes in parameter values (Value/Type).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The event contains the *new* value.

**Non-claims**
- Does not specify the format of the event details.
</details>

### P12 — Describe Unknown Consistent
**Validates:** `SPEC_P06` (Global)
**Layer:** Core

**Claim**
Describing an unknown parameter MUST produce a consistent, non-error response.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Returns an empty descriptor or defined default, does not crash.
</details>

### P13 — List Complete and Stable
**Validates:** `SPEC_P07` (Global)
**Layer:** Tooling

**Claim**
`ListParameters` must be complete and stable across identical queries.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Repeated calls return the same list hash.

**Non-claims**
- Does not assert how lists are returned internally or how often the list is updated.
</details>

### P14 — Deletion Reported via Events
**Validates:** `SPEC_PC08` (Core), `SPEC_P08` (Global)
**Layer:** Policy

**Claim**
Parameter deletion (if supported) is observable via parameter events.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `deleted_parameters` field in the event is populated.

**Non-claims**
- Does not specify how deletion is triggered or managed within the system.
</details>

### P16 — Explicit Declaration Mode
**Validates:** `SPEC_P02` (Global)
**Layer:** Core

**Claim**
Node enforces the declaration mode (Strict vs Lenient).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Behavior matches the `allow_undeclared` configuration.
</details>
### P15 — Parameter Interface Conformity
**Validates:** `SPEC_P01` (Global)
**Layer:** Global
**Claim:** The node exposes the standard ROS 2 parameter interface graph topology.
<details>
<summary>Assertions</summary>
- Services `describe_parameters`, `get_parameter_types`, `get_parameters`, `list_parameters`, `set_parameters` exist.
- Topic `parameter_events` exists.
</details>
