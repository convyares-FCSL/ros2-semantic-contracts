# Ecosystem Contract

This document defines **ecosystem-level semantic constraints** required for
interoperability with the ROS 2 professional stack baseline.

These constraints arise from **widely deployed ecosystem practice** rather than
formal ROS 2 design articles, but are treated as normative for the baseline
profile defined by this repository.

---

## Scope

This contract applies to systems that are expected to interoperate with:

- ROS 2 CLI tooling
- lifecycle managers
- orchestration frameworks
- mature ecosystem stacks (e.g. navigation)

It applies only to **native ROS 2 client library implementations** built on
`rcl` / `rmw`.

---

## Ecosystem Invariants

### Lifecycle Orchestration Compatibility

An implementation MUST expose lifecycle behaviour compatible with external
lifecycle managers.

Lifecycle state, transitions, and failure responses MUST be observable and
controllable via standard ROS 2 lifecycle services.

An implementation that cannot be externally orchestrated is non-compliant.

---

### Liveness and Fault Detection

Active components MUST provide a mechanism for liveness detection such that:

- a crashed or unresponsive component is detectable
- orchestration layers can react deterministically

The specific mechanism (e.g. bond-based monitoring) is ecosystem-defined, but the
**liveness guarantee** is normative.

---

### Action Supersession Semantics

⚠️ **UNVALIDATED (baseline hypothesis)**  
This section describes expected baseline behaviour inferred from ecosystem usage
and implementation inspection. It is not normative until validated by oracle
testing against the Jazzy+rclcpp+Nav2 baseline.

When a new goal supersedes an existing active goal, the previously active goal
MUST transition to a terminal state whose semantic meaning clearly indicates that
it was superseded by a subsequent goal.

In the baseline profile, supersession MUST NOT be represented as `ABORTED`.
Supersession MUST be represented as `CANCELED`, with result or reason information
that indicates goal supersession (preemption).

Using generic failure states that imply internal malfunction to represent goal
supersession is non-compliant.

This constraint reflects ecosystem expectations for safe, predictable behaviour
in long-running and continuously replanned action servers.

---

### Tooling Introspection

A compliant implementation MUST support ecosystem tooling expectations,
including:

- action discovery and status reporting
- parameter event publication
- lifecycle state introspection

These behaviours are validated via oracle and harness testing, not by assertion
alone.

---

### Interoperability over Identity

An implementation MUST behave **indistinguishably** from other compliant
implementations when observed through standard ROS 2 tools.

Semantic compatibility is defined by **observable behaviour**, not internal APIs
or language choice.

---

## Non-Goals

This contract does not:

- prescribe a specific implementation strategy
- mandate a particular language
- require adoption of ecosystem libraries beyond semantic compatibility

---

## Provenance

### Upstream sources
- TBD

### Implementation-defined (rclcpp)
- TBD

### Ecosystem-defined (Nav2)
- TBD

### Project policy
- Baseline interoperability requirements
