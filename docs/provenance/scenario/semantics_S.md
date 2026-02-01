# System Scenario Semantics (S)

This document defines the **semantic claims** made by System oracle scenarios.
Each scenario validates specific invariants in the System Contracts (Composition, Executor, Ecosystem).

It answers the question:
> **What does scenario S (System) assert about the observable world?**

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

### S01 — Load Node Observable
**Validates:** `SPEC_C02` (Composition), `SPEC_C03`
**Layer:** System

**Claim**
The loading of a node must be observable to external systems within the Settling Window.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `load_node` returns success.
- Node appears in `list_nodes`.
- Node topics are visible.

**Non-claims**
- Does not define the exact mechanism of node loading.
</details>

### S02 — Load Failure Clean
**Validates:** `SPEC_C02` (Composition), `SPEC_C05`, `SPEC_C06`
**Layer:** System

**Claim**
If a node fails to load, the system must clean up and leave no "ghost" state.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Failure reason is returned.
- Node does NOT appear in `list_nodes`.

**Non-claims**
- Does not specify how cleanup is managed internally.
</details>

### S03 — Unload Node Destroyed
**Validates:** `SPEC_C03` (Composition), `SPEC_C07`, `SPEC_C08`
**Layer:** System

**Claim**
Unloading a node results in its destruction and removal from the graph.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `unload_node` returns success.
- Node disappears from `list_nodes`.
- Topics/Services are unadvertised.

**Non-claims**
- Does not define the mechanism for node destruction.
</details>

### S04 — Unique Node IDs
**Validates:** `SPEC_C04` (Composition), `SPEC_C10`
**Layer:** System

**Claim**
Every node in the container must have a unique identifier.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- IDs are distinct.
- Reuse is forbidden.

**Non-claims**
- Does not define how IDs are generated.
</details>

### S05 — Concurrent Load Deterministic
**Validates:** `SPEC_C05` (Composition), `SPEC_C13`
**Layer:** System

**Claim**
Concurrent load/unload requests are handled deterministically (serialized or busy-rejected).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Container state remains consistent.
- No race conditions in `list_nodes`.

**Non-claims**
- Does not define the exact method for ordering concurrent loads.
</details>

### S06 — Remaps Apply at Load
**Validates:** `SPEC_C06` (Composition), `SPEC_C15`
**Layer:** System

**Claim**
Remappings supplied at load time must apply immediately to the node instance.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Topics are published on the remapped names, not the code defaults.

**Non-claims**
- Does not define how remaps are applied internally.
</details>

### S07 — Explicit Ownership
**Validates:** `SPEC_E01` (Executor)
**Layer:** System

**Claim**
No callbacks are invoked unless an executor is explicitly spun.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Creating a subscription does not trigger callbacks until `spin()` is called.

**Non-claims**
- Does not define internal tracking methods.
</details>

### S08 — Bounded Spin Modes
**Validates:** `SPEC_E02` (Executor)
**Layer:** System

**Claim**
The system supports bounded spin modes (spin_once, spin_some) for deterministic testing.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `spin_once` returns after processing work.

**Non-claims**
- Does not specify how bounded modes are enforced.
</details>

### S09 — Shutdown Bounded
**Validates:** `SPEC_E03` (Executor)
**Layer:** System

**Claim**
Shutdown unblocks the executor and exits within the Settling Window.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- `context.shutdown()` causes `spin()` to return.

**Non-claims**
- Does not define specific shutdown steps.
</details>

### S10 — Callback Failure Visible
**Validates:** `SPEC_E05` (Executor)
**Layer:** System

**Claim**
Callback failures (exceptions) are visible and do not silently corrupt the executor.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Exceptions propagate or are logged.
- Executor remains stable (or exits cleanly).

**Non-claims**
- Does not define how failures are logged internally.
</details>

### S11 — Lifecycle-Executor Safety
**Validates:** `SPEC_E06` (Executor)
**Layer:** System

**Claim**
Lifecycle transitions do not deadlock the executor.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Calling `change_state` inside a callback does not freeze the system.

**Non-claims**
- Does not define how safety is enforced internally.
</details>

### S12 — Lifecycle Orchestration
**Validates:** `SPEC_ECO01` (Ecosystem)
**Layer:** System

**Claim**
The node is controllable via standard lifecycle managers.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Node responds to standard lifecycle service calls.

**Non-claims**
- Does not specify how orchestration is managed.
</details>

### S13 — Liveness Detection
**Validates:** `SPEC_ECO02` (Ecosystem)
**Layer:** System

**Claim**
A crashed or unresponsive node is detectable.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Bond break or topic silence is observable.

**Non-claims**
- Does not define the frequency of checks.
</details>

### S14 — Tooling Interoperability
**Validates:** `SPEC_ECO04` (Ecosystem), `SPEC_C07`
**Layer:** System

**Claim**
The system is compatible with standard CLI tools (`ros2 node`, `ros2 lifecycle`).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Tools return correct status codes and data.

**Non-claims**
- Does not specify required tooling.
</details>

### S15 — Explicit Ownership/Lifetime
**Validates:** `SPEC_SYS01` (System Contract)
**Layer:** System

**Claim**
Node lifetime is bounded and explicit.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- No global singletons keeping the node alive after destruction.

**Non-claims**
- Does not specify internal lifetime management.
</details>

### S16 — Deterministic Bringup/Teardown
**Validates:** `SPEC_SYS02` (System Contract)
**Layer:** System

**Claim**
Bringup and teardown sequences are deterministic and observable.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Graph state is clean after teardown.

**Non-claims**
- Does not define exact steps for bringup.
</details>

### S17 — No Hidden Execution
**Validates:** `SPEC_SYS03` (System Contract)
**Layer:** System

**Claim**
Execution occurs only via the executor or explicit triggers.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- No background threads spawning unmanaged work.

**Non-claims**
- Does not define detection of hidden execution.
</details>

### S18 — Observable Failure
**Validates:** `SPEC_SYS04` (System Contract)
**Layer:** System

**Claim**
Failures surface as observable states or error codes.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- No silent failures.

**Non-claims**
- Does not define failure classification.
</details>

### S19 — Graph Hygiene
**Validates:** `SPEC_SYS05` (System Contract)
**Layer:** System

**Claim**
The system maintains a clean graph (no orphaned entities).

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- Topic list matches expected active entities.

**Non-claims**
- Does not specify enforcement mechanisms.
</details>