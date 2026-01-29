# Oracle Baseline — ROS 2 Jazzy Professional Stack

This document defines the **explicit oracle baseline** against which semantic contracts in this repository are validated.

It contains **facts only**.
It does not justify correctness, authority, or selection rationale.

---

## Baseline Identity

The baseline profile is defined as:

- ROS Distribution: **Jazzy Jalisco**
- Client Library: **rclcpp**
- Ecosystem Stack: **Nav2**

This profile is referred to throughout the repository as:

> **jazzy+rclcpp+nav2**

---

## Version Pinning

Unless otherwise stated by a specific contract or divergence record, the oracle baseline assumes:

- ROS 2 Jazzy (released distribution)
- rclcpp version shipped with Jazzy
- Nav2 version shipped with Jazzy

Exact package versions, commit SHAs, and Docker image digests are recorded in:

- harness lockfiles
- container manifests
- oracle run logs

Contracts are validated against **observable behaviour**, not source version numbers alone.

---

## Scope of Oracle Authority

The oracle baseline defines expected behaviour only for:

- lifecycle state transitions and services
- action goal, cancel, result, and status semantics
- parameter declaration, mutation, and event emission
- executor-visible scheduling and shutdown behaviour
- ecosystem-level orchestration assumptions relied upon by Nav2

The oracle does **not** define:

- API shape beyond ROS IDL
- implementation strategies
- performance characteristics unless required for correctness
- behaviour of non-native or non-rcl client libraries

---

## Validation Mechanism

Baseline behaviour is validated via:

- containerised execution of Jazzy+rclcpp+Nav2
- harness-driven interaction through standard ROS 2 interfaces
- observation via ROS graph, services, actions, and events

Oracle runs produce:

- pass/fail results per contract clause
- observable traces
- logs sufficient for reproduction

These artifacts are stored alongside the harness output.

---

## Divergence Handling

When observed baseline behaviour diverges from a specified contract:

- a **divergence record** is created
- the divergence is classified as:
  - specification defect
  - upstream regression
  - intentional upstream change

No contract is silently rewritten to match oracle behaviour.

---

## Relationship to Specifications

- Core semantic invariants are defined in `docs/spec/core/*_core.md`
- Global ROS-facing semantics are defined in `docs/spec/*.md`
- System and ecosystem constraints are defined in `docs/spec/system/*`

This document defines **what is tested against**, not **what must be true**.

---

## Reproducibility

Reproduction instructions, container definitions, and harness entrypoints are located under:

- `harness/`
- `tools/`

This document intentionally avoids embedding procedural steps.
