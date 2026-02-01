# Parameters — Core Semantic Contract

**Normativity Class: Core Contract**

This document defines the **normative, transport-agnostic semantic contract** for parameters as enforced by a core semantics engine.
It specifies parameter truth, visibility, decision semantics, and ordering guarantees independently of any ROS transport, executor, or client library.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- parameter store state model
- value types and descriptor enforcement
- set/list/describe decision logic (outcomes)
- logical change records
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- ROS parameter services and wire protocol (see `docs/spec/global/parameters.md`)
- topic publication and QoS
- executor scheduling or threading
- lifecycle state gating
</details>

---

## Core State Model

### SPEC_PC01 — Identity and Case Sensitivity [P01]

Parameters are identified by UTF-8 string names.
- Names are **case-sensitive** and treated opaquely by the core.
- The core MUST NOT fabricate parameters that have not been explicitly set or declared.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC02 — Existence States [P01]

A parameter exists in exactly one of three semantic states:
1.  **Declared:** Exists with a defined type and descriptor.
2.  **Undeclared but Allowed:** Exists implicitly (only if `allow_undeclared` is true).
3.  **Unknown:** Not declared and not implicitly allowed.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

**Notes**
- Unknown parameters MUST NOT appear in list results unless explicitly requested (e.g. via recursive list).

</details>

---

## Values and Typing

### SPEC_PC03 — Type Enforcement [P02, P08]

The core MUST enforce type safety based on the parameter's descriptor.
- **Static Typing (`dynamic_typing=false`):** The value type MUST NOT change after declaration. Setting an incompatible type MUST fail.
- **Dynamic Typing (`dynamic_typing=true`):** The value type MAY change on set.
- Supported types: `NOT_SET`, `BOOL`, `INTEGER`, `DOUBLE`, `STRING`, `BYTE_ARRAY`, and their array variants.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

---

## Decision Logic (Outcomes)

### SPEC_PC04 — Declaration Logic [P03]

Declaration establishes the initial value and descriptor.
- Declaring an already-declared parameter MUST fail deterministically (outcome: `AlreadyDeclared`).
- Declaration order MUST be preserved in change records to ensure deterministic reconstruction of state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC05 — Set Operation Atomicity [P04, P05]

Set operations are atomic transactions.
- A set attempt yields a deterministic outcome: `Successful` or `Failed` (with reason).
- **Atomicity:** Failed operations MUST NOT partially apply. All changes in a request MUST be applied, or none.
- **Read-Only:** If `read_only=true` in the descriptor, any attempt to change the value MUST fail.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC06 — Undeclared Access Logic [P06]

Access to undeclared parameters depends on the node's configuration.
- If `allow_undeclared_parameters` is **false**: Set attempts for unknown parameters MUST fail deterministically.
- If `allow_undeclared_parameters` is **true**: Unknown parameters become implicitly declared upon setting (type inferred from value).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp default behavior

</details>

---

## Change Record Model

### SPEC_PC07 — Deterministic Change Records [P07]

⚠️ **UNVALIDATED (baseline hypothesis)**

The core emits logical **change records** describing modifications.
- Each operation MUST produce exactly one change record (containing one or more parameter changes).
- Ordering within a record MUST reflect the application order.
- **Determinism:** Identical operation sequences MUST produce identical records, independent of wall-clock time.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on event cardinality)
- Baseline: Jazzy+rclcpp implementation

**Notes**
- "Change Record" maps to the `ParameterEvent` message in the global scope, but at the core level, it is a logical struct.

</details>

### SPEC_PC08 — Deletion Support [P14]

⚠️ **UNVALIDATED (baseline hypothesis)**

Deletion support is optional but must be deterministic if present.
- If supported, deletion of a parameter MUST generate a change record indicating removal.
- Deletion of an `Unknown` parameter MUST fail or be a no-op (depending on implementation), but MUST NOT corrupt state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- Baseline: Jazzy+rclcpp

</details>