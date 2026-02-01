# Lifecycle — Core Semantic Contract

**Normativity Class: Core Contract**

This document defines the **normative, transport-agnostic semantic contract** for the ROS 2 managed node lifecycle as enforced by a core semantics engine.
It specifies lifecycle state truth, transition validity, and failure semantics independently of ROS services, executors, or tooling.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- lifecycle state machines
- transition validity
- deterministic state resolution
- error routing
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- ROS lifecycle services and wire protocol (see `docs/spec/global/lifecycle.md`)
- executor scheduling or threading
- timing guarantees
- tooling behaviour
- publication, subscription, or action gating
</details>

---

## Canonical Lifecycle State Model

### SPEC_LC01 — Primary and Auxiliary States [L07]

The core engine MUST enforce a state model consisting of **Primary States** (stable) and **Auxiliary States** (transient).

**Primary States (Stable):**
- `Unconfigured`
- `Inactive`
- `Active`
- `Finalized`

**Auxiliary States (Internal/Transient):**
- `Configuring`
- `Activating`
- `Deactivating`
- `CleaningUp`
- `ShuttingDown`
- `ErrorProcessing`

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)

**Notes**
- Auxiliary states are **internal semantic states** used to resolve transitions. They are not valid stable states.

</details>

---

## Transition Validity

### SPEC_LC02 — Strict Transition Graph [L01, L02]

The core engine MUST enforce a strict transition graph.
- Only defined transitions between primary states are permitted.
- Invalid transitions MUST be rejected deterministically by the engine.
- Rejection is a semantic outcome (invalid request), not a transport error.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)

</details>

---

## Transition Execution Semantics

### SPEC_LC03 — Intermediate State Atomicity [L03]

When a transition is accepted, the core MUST enter the corresponding transition state internally.
- Intermediate states MUST NOT be externally exposed as **stable** lifecycle states.
- The engine MUST NOT accept new transitions while in an intermediate state (atomicity).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)

</details>

---

## Concurrency

### SPEC_LC04 — Single Active Transition [L04, L05]

A lifecycle node MUST process at most one transition at a time.
- Concurrent transition requests MUST be rejected deterministically (Busy).
- Rejection MUST NOT partially apply any transition effects.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)

</details>

---

## ErrorProcessing Semantics

### SPEC_LC05 — Deterministic Error Resolution [L06]

If an error occurs during a transition callback (`on_configure`, `on_activate`, etc.):
- The node MUST enter `ErrorProcessing`.
- `ErrorProcessing` MUST resolve deterministically to a primary state (`Unconfigured` or `Finalized`).

**Baseline Resolution Logic (UNVALIDATED):**
- `on_error` returning `SUCCESS` → Resolves to `Unconfigured`.
- `on_error` returning `FAILURE` or `ERROR` → Resolves to `Finalized`.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- Baseline: Jazzy+rclcpp observed behavior

**Notes**
- This mapping is not normative until validated by the oracle harness. Once validated, it becomes normative for the baseline profile.

</details>