# Composition — System Contract

This document defines **system-level semantic constraints** for ROS 2
node composition required for interoperability with the professional baseline.

It specifies **observable container behaviour** as seen by tools and other nodes,
independent of language or implementation strategy.

Baseline profile:
- ROS 2 Jazzy + rclcpp + Nav2

---

## Scope

This contract applies to:
- in-process node containers
- dynamic load/unload semantics
- container observability
- determinism and failure handling

This contract does **not** define:
- plugin loading mechanisms
- ABI or linking strategy
- client-library APIs
- build or packaging systems

---

## Container interfaces

A composition container exposes services in its node namespace.

Required (baseline tool compatibility):
- `load_node`
- `unload_node`
- `list_nodes`

Exact service definitions are those specified by ROS 2 composition interfaces.
This specification does not redefine the IDL.

---

## Load behaviour

When `load_node` returns success:

- The node becomes observable on the ROS graph within an **implementation-defined bounded time**.
- `list_nodes` reports the node.
- The node’s entities (topics, services, etc.) are active.

If rejected:
- No partial or “ghost” state may appear.
- The failure reason MUST be observable and human-meaningful,
  via service response fields or documented diagnostics.

---

## Unload behaviour

When `unload_node` returns success:

- The node is removed from `list_nodes`.
- All entities are destroyed deterministically.
- The node ceases graph participation.

If rejected:
- No state change occurs.

---

## Identity and stability

- Each loaded node instance MUST receive a unique identifier.
- Identifiers MUST remain stable until unload.
- Reuse of identifiers within a container lifetime is non-compliant.

---

## Concurrency and determinism

Concurrent load/unload requests MUST be handled deterministically:

- Either serialized, or
- Rejected when busy without mutating state.

Busy rejection behaviour MUST be documented.

---

## Parameters and remapping

- Remaps and parameter overrides supplied at load time MUST apply to the node instance.
- Any limitations or deviations MUST be documented.

---

## Tooling compatibility (harness expectation)

Compatibility with `ros2 component` tooling is validated via oracle and harness tests.

This document specifies observable semantics only; tooling behaviour is verified,
not asserted.

---

## Dynamic loading caveat (informative)

Composition semantics are service-based.
Dynamic plugin loading is an implementation detail.

A container may be tool-compatible without runtime loading of shared objects,
provided service semantics are preserved.

---

## Provenance

### Upstream sources
- TBD

### Implementation-defined (rclcpp)
- TBD

### Ecosystem-defined (Nav2)
- TBD

### Project policy
- Baseline composition operability requirements
