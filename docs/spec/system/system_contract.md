# System Contract

**Normativity Class: Baseline System Contract**

These requirements define interoperability constraints for the ROS 2 professional baseline.
They are validated via system-level oracle and harness tests, not core semantic engines.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- native ROS 2 client library implementations built on `rcl` / `rmw`
- systems intended to interoperate with standard ROS 2 tooling and orchestration stacks
- deployments where lifecycle, actions, and parameters are used as control surfaces
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- non-native transports (e.g. rosbridge-only clients)
- experimental or partial ROS 2 integrations unless explicitly stated
</details>

---

## Baseline Interoperability Constraints (BIC)

A **Baseline Interoperability Constraint (BIC)** is a requirement not explicitly defined in upstream ROS 2 specifications, but is necessary for deterministic operation, tooling interoperability, or failure observability in production systems.

Such constraints:
* **MUST** be externally observable.
* **MUST** be testable via oracle/harness.
* **MUST NOT** depend on internal implementation details.
* **ARE REVOCABLE** if oracle evidence shows the production stack violates them without causing ecosystem breakage.

---

## System Invariants

### SPEC_SYS01 — Explicit Ownership and Lifetime [S15]

An implementation MUST make node ownership, construction order, and destruction order explicit.
- Implicit global state, hidden spinners, or background execution that cannot be externally observed or controlled is forbidden.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on threading models)
- REP/RFC: None
- Community: None
- BIC: Definition above (Deterministic execution requirement)

**Notes**
- "Hidden spinners" refers to libraries that spawn threads behind the user's back without an explicit executor.

</details>

### SPEC_SYS02 — Deterministic Bringup and Teardown [S16]

A system MUST support deterministic bringup and teardown sequences with the following properties:
- Components can be initialized in a known order.
- Failures during bringup are observable via standard ROS 2 tools.
- Teardown leaves no residual graph state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: Definition above (Orchestration requirement)

</details>

### SPEC_SYS03 — No Hidden Execution [S17]

⚠️ **UNVALIDATED (baseline hypothesis)**

Execution MUST occur only as a result of:
- executor scheduling
- explicit lifecycle transitions
- externally observable triggers (services, actions, timers)

Background work that continues outside these mechanisms violates system-level semantics.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: Definition above (Real-time analysis requirement)

</details>

### SPEC_SYS04 — Observable Failure [S18]

Failures MUST surface as one of the following:
- lifecycle state transitions (e.g., `deactivating` or `error_processing`)
- action terminal states (`ABORTED`, `CANCELED`)
- explicit errors or rejected operations

Silent failure is forbidden. A system that appears healthy while a component is non-functional violates this contract.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: Definition above (Operability requirement)

</details>

### SPEC_SYS05 — Graph Hygiene [S19]

⚠️ **UNVALIDATED (baseline hypothesis)**

A compliant system MUST reflect its true operational state in the ROS graph at all times:
- No orphaned nodes, publishers, services, or actions may remain after shutdown.
- Graph state MUST be a reliable source of truth for operators and tooling.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: Definition above (Tooling trust requirement)

</details>

### SPEC_SYS06 — Abandoned Goal Cleanup [A17]

⚠️ **UNVALIDATED (baseline hypothesis)**

Action servers MUST eventually clean up goals from clients that have vanished (loss of discovery) or abandoned the goal.
- This cleanup MUST occur within a bounded observation window (e.g., timeout or session expiry).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: Nav2/Ecosystem expectation
- BIC: Definition above (Resource leak prevention)

**Notes**
- "Vanished" implies the client node no longer exists in discovery, or the transport link is severed.

</details>

---

## Relationship to Other Contracts

- Engine-enforceable semantic invariants are defined in `docs/spec/core/*_core.md`
- ROS-facing observable semantics are defined in `docs/spec/global/*.md`
- Ecosystem-level constraints are defined in `docs/spec/system/ecosystem.md`

This document defines **system expectations**, not engine-level semantics.