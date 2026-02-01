# Composition — System Contract

**Normativity Class: Baseline System Contract**

These requirements define interoperability constraints for the ROS 2 professional baseline.
They are validated via system-level oracle and harness tests, not core semantic engines.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- in-process node containers
- dynamic load/unload semantics
- container observability
- determinism and failure handling
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- plugin loading mechanisms
- ABI or linking strategy
- client-library APIs
- build or packaging systems
</details>

---

## Container interfaces

### SPEC_C01 — Standard Composition Services [S21]

A composition container MUST expose the standard ROS 2 composition services in its node namespace to ensure baseline tool compatibility:

* `load_node`
* `unload_node`
* `list_nodes`

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Professional-stack operability requirement)

**Notes**
- Exact service definitions are those specified by ROS 2 composition interfaces. This specification does not redefine the IDL.

</details>

---

## Load behaviour

### SPEC_C02 — Load operation has atomic success/failure semantics [S01, S02]

The `load_node` operation defines a single semantic transaction with the following guarantees.

#### On success
When `load_node` returns success:
- The node MUST become observable on the ROS graph within the **Settling Window** (implementation-defined bounded time).
- The node MUST be reported by `list_nodes`.
- The node’s entities (topics, services, actions) MUST be active and observable.

#### On rejection
If `load_node` is rejected:
- No partial or “ghost” node state may appear on the graph or in `list_nodes`.
- A human-meaningful failure reason MUST be observable via the service response or documented diagnostics.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html) (defines services but not full transactional semantics)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Deterministic orchestration requires atomic load semantics)

**Notes**
- This is an atomic semantic contract. Failure of any subcondition violates SPEC_C02.
- Timing bounds are defined via oracle measurement (`measurement.md`).

</details>

---

## Unload behaviour

### SPEC_C03 — Unload operation has atomic success/failure semantics [S03]

The `unload_node` operation defines a single semantic transaction with the following guarantees.

#### On success
When `unload_node` returns success:
- The node MUST be removed from `list_nodes`.
- All entities owned by the node (topics, services, timers) MUST be destroyed and removed from the graph.
- The node MUST cease graph participation.

#### On rejection
If `unload_node` is rejected:
- The container MUST NOT mutate the node's state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Clean teardown requirement)

</details>

---

## Identity and stability

### SPEC_C04 — Node Identity Contract [S04]

Node identifiers within a container must adhere to the following invariant:
- Each loaded node instance MUST receive a unique identifier.
- Identifiers MUST remain stable from load until unload.
- Reuse of identifiers within a single container lifetime is non-compliant.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html) (Uniqueness)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Safety against race conditions / No Reuse)

**Notes**
- "No Reuse" prevents race conditions in log aggregation and graph monitoring tools.

</details>

---

## Concurrency and determinism

### SPEC_C05 — Concurrency and Determinism [S05]

Concurrent load/unload requests MUST be handled deterministically:
- Requests MUST be either serialized or rejected when busy without mutating state.
- If rejected due to concurrency (busy), the rejection behaviour MUST be documented and observable.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Operability requirement for high-load orchestration)

</details>

---

## Parameters and remapping

### SPEC_C06 — Parameter Application Contract [S06]

Parameter overrides and remappings supplied at load time MUST adhere to the following:
- They MUST apply to the node instance immediately upon instantiation.
- Any limitations or deviations regarding application MUST be documented.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html)
- REP/RFC: None
- Community: None

</details>

---

## Tooling compatibility

### SPEC_C07 — Compatible with Standard CLI [S14]

The container MUST be compatible with standard `ros2 component` CLI tooling.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: Ecosystem expectation
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Ecosystem interoperability)

**Notes**
- This document specifies observable semantics only; tooling behaviour is verified via harness tests, not asserted.

</details>

---

## Dynamic loading caveat (informative)

Composition semantics are service-based.
Dynamic plugin loading is an implementation detail.

A container may be tool-compatible without runtime loading of shared objects,
provided service semantics are preserved.