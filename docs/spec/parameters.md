# Parameters — Global Semantic Specification

Normative status note:
This document may contain sections marked **UNVALIDATED (baseline hypothesis)**.
Such sections describe expected baseline behaviour that is not yet enforced by oracle tests.
They are not part of the normative contract until validated.

---

This document defines the **ROS-facing semantic contract** for parameters:
what clients, tools, and observers can rely on when interacting with a node’s parameters.

It defines observable behaviour, not implementation strategy.

Core semantic invariants are defined in:
- `docs/spec/core/parameter_core.md`

---

## 1) Scope

This specification applies to:
- ROS parameter services
- parameter introspection and tooling
- externally observable parameter behaviour

It does not define:
- internal storage mechanisms
- executor or threading behaviour
- lifecycle gating policies

---

## 2) Names and interfaces

Parameter semantics are exposed via standard ROS 2 parameter services and messages.

Exact service names, message types, and IDL are defined by ROS 2.
This specification does not redefine the IDL.

---

## 3) Declared and undeclared parameters

Nodes MAY operate in:
- **declared-only mode**, or
- **undeclared-allowed mode**

The chosen mode MUST be documented by the implementation.

---

## 4) GetParameters and GetParameterTypes

For both `get_parameters` and `get_parameter_types`:

- Unknown or undeclared parameter names MUST return a value of type `NOT_SET`.
- This behaviour is normative and stable.

This representation allows tooling and clients to reason about unknown parameters
without transport errors or ambiguous failures.

---

## 5) SetParameters and atomic updates

- Parameter updates MUST be atomic per request.
- Partial application MUST NOT be externally observable.
- Rejected updates MUST NOT emit parameter change events.

Atomicity applies to both internal state and externally observable behaviour.

---

## 6) Parameter events

Parameter change events describe **state changes**, not attempts.

> ⚠️ **UNVALIDATED (baseline hypothesis)**
>
> The detailed granularity and cardinality of parameter events are expected
> to match baseline behaviour but have not yet been verified by an oracle harness.
>
> These hypotheses exist to preserve alignment between global observable behaviour
> and the core change-record model.
>
> See `docs/spec/core/parameter_core.md` §5 (Change record and ordering model).

Until validated:
- Observers MUST NOT rely on event count as a proxy for operation count.
- Ordering and content MUST remain deterministic.

---

## 7) DescribeParameters

- Behaviour for unknown parameters MUST be consistent.
- If descriptors are returned for unknown parameters, their semantics MUST be documented.

No default descriptor behaviour is assumed.

---

## 8) ListParameters

> ⚠️ **UNVALIDATED (baseline hypothesis)**
>
> The detailed semantics of prefix filtering and depth limits are expected to match
> Jazzy+rclcpp behaviour but have not yet been verified by an oracle harness.
>
> Until validated, only the following are normative:
>
> - Returned names MUST be complete.
> - Returned names MUST be stable across identical queries.

---

## 9) Deletion semantics

- Parameter deletion support is optional.
- If supported, deletion MUST be documented.
- Deleted parameters MUST be reported via parameter events.

No deletion behaviour is assumed by default.

---

## Provenance

### Upstream sources
- TBD

### Implementation-defined (rclcpp)
- TBD

### Ecosystem-defined (Nav2)
- TBD

### Project policy
- Stable observable parameter semantics
