# System Contract

This document defines **system-level contracts**, not core semantic engine
contracts, required for a professional ROS 2 deployment baseline.

It does not define APIs or semantic engines.
It defines **system invariants** that must hold for a ROS 2 system to be
operable, observable, and orchestratable under real deployment conditions.

These constraints are **normative** for the baseline profile defined by this
repository and are validated via system-level harnesses, not by a pure semantic
engine.

---

## Scope

This contract applies to:

- native ROS 2 client library implementations built on `rcl` / `rmw`
- systems intended to interoperate with standard ROS 2 tooling and orchestration
  stacks
- deployments where lifecycle, actions, and parameters are used as control
  surfaces

This contract does **not** apply to:

- non-native transports (e.g. rosbridge-only clients)
- experimental or partial ROS 2 integrations unless explicitly stated

---

## System Invariants

### Explicit Ownership and Lifetime

An implementation MUST make node ownership, construction order, and destruction
order explicit.

Implicit global state, hidden spinners, or background execution that cannot be
externally observed or controlled is forbidden.

---

### Deterministic Bringup and Teardown

A system MUST support deterministic bringup and teardown sequences such that:

- components can be initialized in a known order
- failures during bringup are observable and bounded
- teardown leaves no residual graph state

Bringup and shutdown semantics MUST be externally observable via standard ROS 2
graph and lifecycle tools.

---

### No Hidden Execution

Execution MUST occur only as a result of:

- executor scheduling
- explicit lifecycle transitions
- externally observable triggers (services, actions, timers)

Background work that continues outside these mechanisms violates system-level
semantics.

---

### Observable Failure

Failures MUST surface as:

- lifecycle state transitions
- action terminal states
- explicit errors or rejected operations

Silent failure is forbidden.
A system that appears healthy while a component is non-functional violates this
contract.

---

### Graph Hygiene

A compliant system MUST:

- leave no orphaned nodes, publishers, services, or actions after shutdown
- reflect its true operational state in the ROS graph at all times

Graph state MUST be a reliable source of truth for operators and tooling.

---

## Relationship to Other Contracts

- Engine-enforceable semantic invariants are defined in `docs/spec/core/*_core.md`
- ROS-facing observable semantics are defined in `docs/spec/*.md`
- Ecosystem-level constraints are defined in `docs/spec/ecosystem.md`

This document defines **system expectations**, not engine-level semantics.

---

## Provenance

### Upstream sources
- TBD

### Implementation-defined (rclcpp)
- TBD

### Ecosystem-defined (Nav2)
- TBD

### Project policy
- Professional-stack operability requirements
