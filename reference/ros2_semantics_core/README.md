# ros2_semantics_core

`ros2_semantics_core` is a **transport-agnostic reference semantics engine** for ROS 2 semantic contracts.

This crate implements **core semantic invariants** as pure logic (types + state transitions), without any ROS graph,
middleware, DDS, executor, threading, or client-library bindings. It is designed to be embedded by integration layers
(rclcpp/rclpy/rclrs/etc.) and validated by contract tests and oracle harnesses.

### Authority

Semantic truth is defined by the **specifications in this repository**, not by this crate:
- Core invariants: `docs/spec/core/*_core.md`
- ROS-facing semantics: `docs/spec/*.md`
- System/ecosystem contracts: `docs/spec/system/*.md`

This crate exists to provide an **executable reference** for those contracts.

### What is (and is not) here

**This crate includes:**
- Deterministic, testable semantics for core behaviors (e.g., parameters atomicity and change records)
- Explicit outcome types (success/failure reasons) rather than hidden side effects

**This crate does not include:**
- ROS services/topics/actions implementations
- Executor scheduling, callback groups, timing assumptions
- DDS/RMW behavior
- CLI/tooling behavior (validated by harnesses, not asserted here)

### Parameters quick example

```rust
use ros2_semantics_core::parameters::{Descriptor, ParameterStore, Type, Value};

let mut store = ParameterStore::new(false);

// Declare a parameter
store
  .declare("threshold", Type::Double, Value::Double(0.5), Descriptor::default())
  .unwrap();

// Atomically update
let (result, record) = store.set_parameters_atomically(vec![
  ("threshold".to_string(), Value::Double(1.0)),
]);

assert!(result.success);
assert_eq!(record.changed_parameters.len(), 1);
