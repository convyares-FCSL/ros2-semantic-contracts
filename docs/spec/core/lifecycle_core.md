# Lifecycle — Core Semantic Contract

Normative status note:
This document may contain sections marked **UNVALIDATED (baseline hypothesis)**.
Such sections describe expected baseline behaviour that is not yet enforced by oracle tests.
They are not part of the normative contract until validated.

---

This document defines the **normative, transport-agnostic semantic contract** for the ROS 2
managed node lifecycle as enforced by a core semantics engine.

It specifies lifecycle state truth, transition validity, and failure semantics independently
of ROS services, executors, or tooling.

Canonical global specification:
- `docs/spec/lifecycle.md`

---

## 1) Scope and non-goals

This contract applies to:
- lifecycle state machines
- transition validity
- deterministic state resolution
- error routing

This contract explicitly excludes:
- ROS lifecycle services and message types
- executor scheduling or threading
- timing guarantees
- tooling behaviour
- publication, subscription, or action gating

---

## 2) Canonical lifecycle state model

Primary states:
- Unconfigured
- Inactive
- Active
- Finalized

Auxiliary states:
- Configuring
- Activating
- Deactivating
- CleaningUp
- ShuttingDown
- ErrorProcessing

Auxiliary states are **internal semantic states** used to resolve transitions.

---

## 3) Transition validity

- Only defined transitions between primary states are permitted.
- Invalid transitions MUST be rejected deterministically.
- Rejection is a semantic outcome, not a transport error.

---

## 4) Transition execution semantics

### Intermediate-state correctness

When a transition is accepted, the core MUST enter the corresponding
transition state internally (**not externally observable**) until it resolves
to a primary state or ErrorProcessing.

Intermediate states MUST NOT be externally observable as stable lifecycle states.

---

## 5) Busy and concurrent transitions

- A lifecycle node MUST process at most one transition at a time.
- Concurrent transition requests MUST be rejected deterministically.
- Rejection MUST NOT partially apply any transition effects.

---

## 6) ErrorProcessing semantics

If an error occurs during a transition:
- The node MUST enter ErrorProcessing.
- ErrorProcessing MUST resolve deterministically to a primary state.

> ⚠️ **UNVALIDATED (baseline hypothesis)**
>
> Baseline expectation (Jazzy+rclcpp):
> - on_error returning SUCCESS resolves to Unconfigured
> - on_error returning FAILURE or ERROR resolves to Finalized
>
> This mapping is not normative until validated by the oracle harness.
> Once validated, it becomes normative for the baseline profile only.

---

## 7) Invariants (testable)

The core MUST enforce:
- Single active transition at a time
- Deterministic acceptance or rejection of transitions
- No partial transition effects
- Deterministic resolution from ErrorProcessing
- No externally observable intermediate states

---

## Provenance

### Upstream sources
- TBD

### Implementation-defined (rclcpp)
- TBD

### Ecosystem-defined (Nav2)
- TBD

### Project policy
- None
