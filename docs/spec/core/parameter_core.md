# Parameters — Core Semantic Contract

Normative status note:
This document may contain sections marked **UNVALIDATED (baseline hypothesis)**.
Such sections describe expected baseline behaviour that is not yet enforced by oracle tests.
They are not part of the normative contract until validated.

---

This document defines the **normative, transport-agnostic semantic contract** for parameters as enforced by a
core semantics engine.

It specifies parameter truth, visibility, decision semantics, and ordering guarantees independently of any ROS
transport, executor, or client library.

It does **not** define ROS graph wiring, ROS parameter services, message types, or event transport.

Canonical global specification:
- `docs/spec/parameters.md`

---

## 1) Scope and non-goals

This contract applies to a core parameter semantics engine and its logical API surface, including:
- a parameter store
- value types
- descriptors
- set/list/describe outcomes
- logical change records

This contract explicitly excludes:
- ROS parameter services and message types
- topic publication and QoS
- executors, threading, or scheduling
- lifecycle state gating
- wall-clock time or timestamps

Adapters are responsible for exposing ROS-facing behaviour using these semantics.

---

## 2) Core state model

### Parameter identity
- Parameters are identified by UTF-8 string names.
- Names are case-sensitive and treated opaquely by the core.

### Parameter existence
A parameter may be in one of three semantic states:
- **Declared** — exists with defined type and descriptor
- **Undeclared but allowed** — exists implicitly if undeclared parameters are permitted
- **Unknown** — not declared and not implicitly allowed

Unknown parameters MUST NOT be fabricated.

---

## 3) Values and typing

Supported value set:
- NOT_SET, BOOL, INTEGER, DOUBLE, STRING,
  BYTE_ARRAY, BOOL_ARRAY, INTEGER_ARRAY, DOUBLE_ARRAY, STRING_ARRAY

Typing rules:
- If `dynamic_typing = false`, the value type MUST NOT change after declaration.
- If `dynamic_typing = true`, the value type MAY change on set.
- Setting a value with an incompatible type MUST fail deterministically.

---

## 4) Decision envelopes (outcomes, not errors)

Parameter operations return **outcomes**, not exceptions.
Exceptional failures are reserved for invariant violations or API misuse.

### 4.1) Declaration
- Declaring an already-declared parameter MUST fail deterministically.
- Declaration establishes initial value and descriptor.
- Declaration order MUST be preserved in change records.

### 4.2) Setting parameters
- Each set attempt yields an outcome: success or failure, with optional reason.
- Failed operations MUST NOT partially apply.
- Atomic operations MUST apply all changes or none.

Read-only enforcement:
- If `read_only = true`, any attempt to change the parameter MUST fail.

### 4.3) Unknown parameters
- If undeclared parameters are not allowed, set attempts MUST fail deterministically.
- If undeclared parameters are allowed, parameters become implicitly declared with inferred type.
- Unknown parameters MUST NOT appear in results unless explicitly requested.

---

## 5) Change record and ordering model

> ⚠️ **UNVALIDATED (baseline hypothesis)**
>
> The following rules are expected based on inspection of baseline implementation
> and ecosystem usage, but have not yet been verified by an oracle harness
> against the Jazzy+rclcpp+Nav2 baseline.
>
> These rules are not normative until validated.

The core emits logical **change records** describing parameter modifications.

Hypothesised rules:
- Each operation produces exactly one change record.
- Records may include:
  - newly created parameters
  - changed parameters
  - deleted parameters (if supported)
- Ordering within a record reflects application order.

Determinism:
- Ordering MUST NOT depend on wall-clock time.
- Identical operation sequences MUST produce identical records.

Deletion note:
Deletion support is not part of the core contract unless explicitly validated.
Deletion records are included only if deletion is supported by the baseline.

---

## 6) Invariants (testable)

The core MUST enforce:
- No fabricated parameters
- Deterministic failure for invalid operations
- Atomicity of atomic updates
- Read-only enforcement
- Dynamic typing rules
- Deterministic change ordering
- No side effects on failure

---

## 7) Traceability map

| Global spec heading (`docs/spec/parameters.md`) | Core clause(s) |
| --- | --- |
| Parameter declaration | 2, 4.1 |
| Parameter typing | 3, 6 |
| Set parameters | 4.2, 6 |
| Atomic updates | 4.2, 6 |
| Unknown parameters | 4.3, 6 |
| Parameter events | 5 |

---

## Provenance

### Upstream sources
- [ROS 2 Docs: About Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic-Concepts/About-Parameters.html)

### Implementation-defined (rclcpp)
- [ROS 2 Guides: Using callback groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)
- [rclcpp Source: node_parameters.cpp](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp)

### Ecosystem-defined (Nav2)
- TBD: needs oracle validation (see docs/provenance/oracle_plan.md)

### Project policy
- None
