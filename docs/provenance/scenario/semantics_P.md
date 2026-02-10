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
**Validates:** `SPEC_PC06` (Core)
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

**Scope note**
- Core↔Global composite: SPEC_PC07 defines the logical change-record model; SPEC_P05 defines the wire-observable event semantics. Both are exercised because the parameter event IS the change record exposed over the service layer.
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

**Scope note**
- Core↔Global composite: SPEC_PC03 defines type enforcement at the decision layer; SPEC_P03 defines the NOT_SET wire response. Both are exercised because the NOT_SET response is the observable manifestation of an unknown parameter's type state.
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

**Scope note**
- Core↔Global composite: SPEC_PC08 defines deletion at the decision layer; SPEC_P08 defines deletion reporting on the wire. Both are exercised because event emission is the observable consequence of the deletion decision.
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

### P17 — Non-Atomic Batch Callback Rejection
**Validates:** `SPEC_PC13` (Core), `SPEC_P09` (Global)
**Layer:** Global

**Claim**
When `set_parameters` (non-atomic batch) is used with callback rejection, updates are evaluated per-parameter and partial application is possible.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback registration succeeds.
- Non-atomic batch operation executes and returns a batch result envelope.
- Accepted key applies (`p17_param_a = a1`) while rejected key remains unchanged (`p17_param_b = b0`).

**Non-claims**
- Current scenario does not assert fairness/latency properties of callback execution under load.
</details>

### P18 — Atomic Batch Callback Rejection
**Validates:** `SPEC_PC05` (Core), `SPEC_P04` (Global)
**Layer:** Global

**Claim**
When `set_parameters_atomically` is used with callback rejection, the batch is rejected as a unit (`successful=false`).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback registration succeeds.
- Atomic batch set reports `successful=false` under configured rejection.
- Pre-existing values are preserved (`p18_param_a = a0`, `p18_param_b = b0`) after rejection.

**Non-claims**
- Current scenario does not assert callback ordering relative to other callback types.
</details>

### P19 — Pre-Set Callback Removal Semantics
**Validates:** discovery extension for pre-set callback behavior
**Layer:** Global

**Claim**
A pre-set callback can transform the incoming batch (including removing keys) before apply.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Pre-set callback registration succeeds.
- Batch operation executes after callback installation.
- Both target parameters remain queryable.

**Non-claims**
- Current scenario does not assert value-level fate for removed keys in expectation patterns.
- Current scenario does not assert callback ordering relative to other callback types.
</details>

### P20 — Parameter Event Cardinality for Atomic vs Non-Atomic Batch
**Validates:** issue-signature probe for `rclcpp#1970`, `SPEC_P05` event envelope
**Layer:** Global

**Claim**
Atomic and non-atomic batch APIs produce different event cardinality signatures on `/parameter_events`.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Atomic batch update emits exactly one grouped `parameter_event` containing all changed keys.
- Non-atomic batch update emits per-parameter events (one changed key per event) for the tested 3-key batch.
- Event drains show expected counts in the observation windows.

**Non-claims**
- Does not assert global scheduler fairness or cross-node ordering guarantees.
- Does not assert latency bounds for event publication.
</details>

### P21 — Descriptor Tracks Dynamic Value Type
**Validates:** `SPEC_PC09` (Core)
**Layer:** Core

**Claim**
For dynamic-typed parameters, descriptor `type` reflects current value type after a successful type-changing set.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Descriptor type is string (`4`) after initial string declaration.
- Descriptor type is integer (`2`) after successful integer set.

**Non-claims**
- Does not assert descriptor updates for static-typed parameters.
- Does not assert range/constraint metadata synchronization.
</details>

### P22 — Explicit Descriptor Type Enforcement
**Validates:** `SPEC_PC10` (Core)
**Layer:** Core

**Claim**
Declaring with explicit `ParameterDescriptor.type` should preserve that declared type.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Declaration succeeds with explicit descriptor type.
- Describe call reports the expected descriptor type for the matching-value test case.

**Non-claims**
- Current scenario is a matching-value case (`value` type == descriptor type) and does not prove mismatch enforcement behavior.
</details>

### P23 — Declaration Without Default Value
**Validates:** `SPEC_PC11` (Core)
**Layer:** Core

**Claim**
Explicitly typed declaration without a default value is permitted, and initial read state is `NOT_SET`.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Declaration succeeds with descriptor type and no default value.
- Subsequent read returns declared=true with type `NOT_SET`.

**Non-claims**
- Does not assert launch-time override behavior (YAML/CLI injection path).
</details>

### P24 — GetParameterTypes 1:1 Response Conformance
**Validates:** `SPEC_PC12` (Core, official-contract conformance)
**Layer:** Core

**Claim**
`GetParameterTypes` should preserve 1:1 request-to-response mapping, including undeclared names as `NOT_SET` entries.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Declared-only request returns 1:1 mapping (`request_count == count == 1`).
- Mixed declared/undeclared request is expected to return 1:1 mapping (`request_count == count == 3`, types `[4,0,0]`).

**Non-claims**
- Does not normalize backend error text.

**Current measured state**
- In pinned Docker Jazzy+Humble runs in this repository, both backends fail this conformance claim for mixed requests (they return `NotDeclared` with empty type vector).
</details>

### P25 — Concurrent Read During Non-Atomic Batch
**Validates:** `SPEC_PC14` (Core), `SPEC_P09` (Global)
**Layer:** Core

**Claim**
Under repeated single-call `set_parameters` (non-atomic) updates with concurrent reads, mixed old/new cross-key snapshots are observed on the measured baseline path.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Probe runs with configured trial depth (`trial_count`) and read cadence (`reader_sleep_us`).
- `sample_sufficient=true` must hold (`snapshot_count >= min_snapshots`).
- Mixed visibility is observed (`partial_state_observed=true`) on the measured path.
- Operation completes successfully.

**Non-claims**
- Scenario does not establish universal impossibility/guarantee across all schedulers.
</details>

### P26 — Type Coercion Matrix (Static vs Dynamic)
**Validates:** `SPEC_PC03` (Core), `SPEC_PC15` (Core)
**Layer:** Core

**Claim**
Non-happy-path coercion attempts on statically typed parameters are rejected (no implicit scalar coercion on tested paths), while dynamic-typed control transitions are accepted.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Static integer parameter rejects explicit `double` and `bool` set attempts.
- Static boolean parameter rejects explicit `integer` set attempt.
- After rejections, static parameters retain original type/value.
- Dynamic-typed numeric parameter accepts explicit `integer -> double -> integer` transitions.

**Non-claims**
- Does not assert array coercion behavior.
- Does not assert all historical distro/version paths; scenario records Jazzy behavior measured in this repository.
</details>

### P27 — Standard Batch Partial Application Check
**Validates:** `SPEC_PC13` (Core), `SPEC_P09` (Global)
**Layer:** Core

**Claim**
`set_parameters` uses sequential non-atomic semantics: if a later element is rejected, earlier accepted elements remain applied.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback registration succeeds.
- Non-atomic batch request returns a mixed/failed envelope.
- Earlier key remains updated while rejected key remains unchanged.

**Non-claims**
- Does not assert callback fairness or latency under load.
- Does not assert internal locking strategy.
</details>

### P28 — Atomic Batch Transaction Check
**Validates:** `SPEC_PC05` (Core), `SPEC_P04` (Global)
**Layer:** Core

**Claim**
`set_parameters_atomically` is transactional: any callback rejection fails the request and preserves all prior values.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback registration succeeds.
- Atomic batch set reports `successful=false`.
- All keys in the request retain their pre-request values.

**Non-claims**
- Does not assert callback ordering relative to pre/post-set callbacks.
</details>

### P29 — Callback Storage Old-Value Visibility
**Validates:** `SPEC_PC16` (Core)
**Layer:** Core

**Claim**
During validation callback execution, storage reads still expose the old committed value for the target parameter.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback mode registration succeeds with explicit target/expected-old contract.
- Set operation succeeds under the old-value assertion callback.
- Final committed value reflects the requested update.

**Non-claims**
- Does not assert post-set callback timing.
- Does not assert cross-thread observer timing.
</details>

### P30 — External Visibility After Success Only
**Validates:** `SPEC_PC16` (Core), `SPEC_P10` (Global)
**Layer:** Global

**Claim**
Rejected updates are externally silent; parameter events become visible only after successful commit.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Rejected atomic batch emits zero events in the observation window.
- State remains unchanged after rejected request.
- Successful follow-up atomic batch emits one change event with committed value.

**Non-claims**
- Does not assert global scheduler fairness.
- Does not assert latency bounds beyond observation windows.
</details>

### P33 — Standard Batch Partial Application (Type Failure)
**Validates:** `SPEC_PC16` (Core)
**Layer:** Core

**Claim**
For standard/non-atomic batch updates, a later type-validation failure does not roll back earlier successful elements; callback-time storage for the first key still exposes the old committed value.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback registration in old-value assertion mode succeeds.
- Non-atomic batch request reports `successful=false` for mixed outcome.
- First key commits (`p33_param_a = 11`) while failing second key remains unchanged (`p33_param_b = 20`).

**Non-claims**
- Does not assert cross-thread visibility/locking behavior.
- Does not assert post-set callback timing.
</details>

### P34 — Atomic Batch Transaction Integrity (Type Failure)
**Validates:** `SPEC_PC16` (Core), `SPEC_P10` (Global)
**Layer:** Core

**Claim**
For atomic batch updates, any type-validation failure rejects the request as a unit and preserves all pre-request values.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Callback registration in old-value assertion mode succeeds.
- Atomic batch request reports `successful=false`.
- Both keys retain pre-request values (`p34_param_a = 10`, `p34_param_b = 20`).

**Non-claims**
- Does not assert cross-thread visibility/locking behavior.
- Does not assert post-set callback timing.
</details>

### P35 — Concurrent Read During Atomic Batch (Control)
**Validates:** `SPEC_PC14` (Core), `SPEC_P10` (Global)
**Layer:** Core

**Claim**
Under repeated single-call `set_parameters_atomically` updates with concurrent reads, mixed old/new cross-key snapshots are not observable on the measured path.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Probe runs in `api=atomic` mode with configured trial depth.
- `sample_sufficient=true` must hold (`snapshot_count >= min_snapshots`).
- Mixed visibility is not observed (`partial_state_observed=false`) on the measured path.
- Operation completes successfully.

**Non-claims**
- Scenario does not claim universal scheduler behavior outside measured conditions.
</details>

### P31 — No-Default Declaration With Startup Override
**Validates:** `SPEC_PC11` (Core)
**Layer:** Core

**Claim**
When a CLI startup override (`--ros-args -p key:=value`) exists for a statically typed parameter, explicit typed declaration without default value should bind the override value instead of throwing.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Declaration succeeds for typed no-default parameter under CLI startup override.
- Readback returns declared and set with the override value.

**Non-claims**
- Does not assert YAML file transport source; scenario targets explicit CLI argument path.
- Does not assert behavior when no override is supplied.

**Current measured state**
- Pinned Docker Jazzy+Humble:
  - `backend_prod`: PASS (both distros)
  - `backend_py`: PASS on Humble, FAIL on Jazzy (`Cannot declare a statically typed parameter with default value of type PARAMETER_NOT_SET`)
</details>

### P32 — Startup Declaration Event Batching
**Validates:** `SPEC_P05` (Global) issue-equivalent path for startup declarations
**Layer:** Global

**Claim**
Startup-time `declare_parameter` calls should be observable as one batched parameter event for the startup node.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Startup probe op reports declaration count and captured startup event signature.
- Batched single-event signature is required (`event_count=1`, `max_keys_in_single_event=declare_count`).

**Non-claims**
- Does not assert global ordering relative to unrelated nodes' parameter events.
- Does not assert non-startup set APIs; those are covered by `P20`, `P27`-`P30`.

**Current measured state**
- In pinned Docker Jazzy+Humble runs in this repository (`backend_prod` and `backend_py`), this claim fails: startup declaration calls were observed as three single-key events, not one batched event.
</details>

### P36 — Integer Descriptor Range Enforcement
**Validates:** `SPEC_PC17` (Core)
**Layer:** Core

**Claim**
If an integer parameter descriptor defines an integer range, out-of-range set attempts are rejected and the last accepted value remains.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Declaration with descriptor `integer_range=[{from_value:0,to_value:10,step:1}]` succeeds.
- `describe_param` returns the configured integer range.
- In-range set (`8`) succeeds.
- Out-of-range set (`11`) fails.
- Final readback remains the last accepted value (`8`).

**Non-claims**
- Does not assert floating-point range behavior.
- Does not assert cross-distribution equivalence beyond measured matrix evidence.

**Current measured state**
- Pinned Docker Jazzy+Humble (`backend_prod`, `backend_py`): PASS.
</details>

### P37 — Floating Descriptor Range Enforcement
**Validates:** `SPEC_PC17` (Core)
**Layer:** Core

**Claim**
If a floating-point parameter descriptor defines a floating-point range, out-of-range set attempts are rejected and the last accepted value remains.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Declaration with descriptor `floating_point_range=[{from_value:0.0,to_value:1.0,step:0.1}]` succeeds.
- `describe_param` returns the configured floating-point range.
- In-range set (`0.9`) succeeds.
- Out-of-range set (`1.5`) fails.
- Final readback remains the last accepted value (`0.9`).

**Non-claims**
- Does not assert integer range behavior.
- Does not assert step-rounding edge behavior outside measured values.

**Current measured state**
- Pinned Docker Jazzy+Humble (`backend_prod`, `backend_py`): PASS.
</details>
