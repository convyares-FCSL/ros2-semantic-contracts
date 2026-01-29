/*
# Parameter Core Spec Conformance Audit (Core Only)

This file is a black-box contract suite for `ros2_semantics_core::parameters`, analogous to
the action/lifecycle core contract tests.

It records a short spec↔code↔tests audit summary for:

- `docs/spec/core/parameter_core.md`
- (ROS-facing permanence rules referenced by) `docs/spec/parameters.md`

This suite is intentionally **core-only**:
- It enforces deterministic, transport-agnostic invariants.
- It does **not** assert oracle/harness expectations (e.g., `/parameter_events` message granularity).

## ✅ Fully covered clauses (core-level)

### Parameter core (`docs/spec/core/parameter_core.md`)

- Declared vs allow-undeclared behaviour (no ghost parameters in declared-only mode)
  - Enforced by: `ParameterStore::{declare,prepare_set,set_parameter,set_parameters_atomically}`
  - Proven by: `declared_only_rejects_undeclared_without_side_effects`

- Type correctness and dynamic typing envelope
  - Enforced by: `ParameterStore::{declare,prepare_set}`
  - Proven by: `type_mismatch_is_rejected_when_dynamic_typing_disabled`,
             `type_change_is_allowed_when_dynamic_typing_enabled`

- Read-only descriptor semantics
  - Enforced by: `ParameterStore::prepare_set` (read_only check)
  - Proven by: `read_only_rejects_and_emits_no_event`

- Atomic set (all-or-nothing) + no partial application + event suppression on rejection
  - Enforced by: `ParameterStore::set_parameters_atomically`
  - Proven by: `atomic_set_rejects_without_partial_apply`,
             `atomic_rejection_emits_no_event_record`

- Unknown-name representation stability for Get/GetTypes
  - Enforced by: `ParameterStore::{get,get_types}`
  - Proven by: `unknown_get_returns_not_set`,
             `unknown_get_types_returns_not_set`

- Deletion via NOT_SET is rejected (no deletion support in this core store)
  - Enforced by: `ParameterStore::prepare_set` (reject `Value::NotSet`)
  - Proven by: `deletion_not_supported_is_rejected_and_no_event`

## ⚠️ Not covered here by design

- Exact `/parameter_events` message cardinality/granularity and timing:
  this is harness/oracle territory, not core.

## ❌ Missing enforcement

- None identified for the core-only model implemented by `ParameterStore`.
*/

use ros2_semantics_core::parameters::{Descriptor, EventRecord, ParameterStore, Type, Value};

fn declared_store() -> ParameterStore {
    ParameterStore::new(false)
}

fn undeclared_store() -> ParameterStore {
    ParameterStore::new(true)
}

fn default_desc() -> Descriptor {
    Descriptor::default()
}

#[test]
fn unknown_get_returns_not_set() {
    let store = declared_store();
    let vals = store.get(&["missing".to_string()]);
    assert_eq!(vals, vec![Value::NotSet]);
}

#[test]
fn unknown_get_types_returns_not_set() {
    let store = declared_store();
    let tys = store.get_types(&["missing".to_string()]);
    assert_eq!(tys, vec![Type::NotSet]);
}

#[test]
fn declared_only_rejects_undeclared_without_side_effects() {
    let mut store = declared_store();

    // Undeclared set is rejected.
    let (res, ev) = store.set_parameter("x".to_string(), Value::Integer(1));
    assert!(!res.success);
    assert!(ev.is_empty());

    // Still unknown afterwards (no ghost creation).
    assert_eq!(store.get(&["x".to_string()]), vec![Value::NotSet]);
    assert_eq!(store.get_types(&["x".to_string()]), vec![Type::NotSet]);
}

#[test]
fn allow_undeclared_creates_parameter_and_emits_new_event() {
    let mut store = undeclared_store();

    let (res, ev) = store.set_parameter("x".to_string(), Value::Integer(1));
    assert!(res.success);
    assert_eq!(store.get(&["x".to_string()]), vec![Value::Integer(1)]);
    assert_eq!(store.get_types(&["x".to_string()]), vec![Type::Integer]);

    // Should be recorded as "new".
    assert_eq!(ev.new_parameters.len(), 1);
    assert_eq!(ev.changed_parameters.len(), 0);
    assert_eq!(ev.deleted_parameters.len(), 0);
    assert_eq!(ev.new_parameters[0].name, "x");
}

#[test]
fn type_mismatch_is_rejected_when_dynamic_typing_disabled() {
    let mut store = declared_store();
    store
        .declare("p", Type::Integer, Value::Integer(1), default_desc())
        .expect("declare should succeed");

    let (res, ev) = store.set_parameter("p".to_string(), Value::String("nope".into()));
    assert!(!res.success);
    assert!(ev.is_empty());

    // Value remains unchanged.
    assert_eq!(store.get(&["p".to_string()]), vec![Value::Integer(1)]);
    assert_eq!(store.get_types(&["p".to_string()]), vec![Type::Integer]);
}

#[test]
fn type_change_is_allowed_when_dynamic_typing_enabled() {
    let mut store = declared_store();
    let mut d = default_desc();
    d.dynamic_typing = true;

    store
        .declare("p", Type::Integer, Value::Integer(1), d)
        .expect("declare should succeed");

    let (res, ev) = store.set_parameter("p".to_string(), Value::String("ok".into()));
    assert!(res.success);

    // Type and value update.
    assert_eq!(
        store.get(&["p".to_string()]),
        vec![Value::String("ok".into())]
    );
    assert_eq!(store.get_types(&["p".to_string()]), vec![Type::String]);

    // This is a change on an existing parameter.
    assert_eq!(ev.new_parameters.len(), 0);
    assert_eq!(ev.changed_parameters.len(), 1);
    assert_eq!(ev.deleted_parameters.len(), 0);
}

#[test]
fn read_only_rejects_and_emits_no_event() {
    let mut store = declared_store();
    let mut d = default_desc();
    d.read_only = true;

    store
        .declare("ro", Type::Integer, Value::Integer(1), d)
        .expect("declare should succeed");

    let (res, ev) = store.set_parameter("ro".to_string(), Value::Integer(2));
    assert!(!res.success);
    assert!(ev.is_empty());

    // Value remains unchanged.
    assert_eq!(store.get(&["ro".to_string()]), vec![Value::Integer(1)]);
}

#[test]
fn deletion_not_supported_is_rejected_and_no_event() {
    let mut store = undeclared_store();

    // Attempt "deletion" is rejected.
    let (res, ev) = store.set_parameter("x".to_string(), Value::NotSet);
    assert!(!res.success);
    assert!(ev.is_empty());

    // Not created.
    assert_eq!(store.get(&["x".to_string()]), vec![Value::NotSet]);
    assert_eq!(store.get_types(&["x".to_string()]), vec![Type::NotSet]);
}

#[test]
fn atomic_set_rejects_without_partial_apply() {
    let mut store = declared_store();
    store
        .declare("a", Type::Integer, Value::Integer(1), default_desc())
        .expect("declare should succeed");
    store
        .declare("b", Type::Integer, Value::Integer(10), default_desc())
        .expect("declare should succeed");

    // Second update is invalid (type mismatch) -> whole batch rejected.
    let (res, ev) = store.set_parameters_atomically(vec![
        ("a".to_string(), Value::Integer(2)),
        ("b".to_string(), Value::String("bad".into())),
    ]);
    assert!(!res.success);
    assert!(ev.is_empty());

    // No partial apply.
    assert_eq!(store.get(&["a".to_string()]), vec![Value::Integer(1)]);
    assert_eq!(store.get(&["b".to_string()]), vec![Value::Integer(10)]);
}

#[test]
fn atomic_rejection_emits_no_event_record() {
    let mut store = declared_store();
    store
        .declare("a", Type::Integer, Value::Integer(1), default_desc())
        .expect("declare should succeed");

    let (res, ev) = store.set_parameters_atomically(vec![
        ("a".to_string(), Value::Integer(2)),
        ("missing".to_string(), Value::Integer(3)), // undeclared -> rejects (declared-only)
    ]);

    assert!(!res.success);
    assert!(ev.is_empty());

    // Confirm a not partially updated.
    assert_eq!(store.get(&["a".to_string()]), vec![Value::Integer(1)]);
}

#[test]
fn atomic_success_applies_all_and_emits_only_successful_changes() {
    let mut store = declared_store();
    store
        .declare("a", Type::Integer, Value::Integer(1), default_desc())
        .expect("declare should succeed");
    store
        .declare("b", Type::Integer, Value::Integer(10), default_desc())
        .expect("declare should succeed");

    let (res, ev) = store.set_parameters_atomically(vec![
        ("a".to_string(), Value::Integer(2)),
        ("b".to_string(), Value::Integer(11)),
    ]);

    assert!(res.success);
    assert_eq!(store.get(&["a".to_string()]), vec![Value::Integer(2)]);
    assert_eq!(store.get(&["b".to_string()]), vec![Value::Integer(11)]);

    // Expect 2 changed parameters recorded (both existing).
    assert_eq!(ev.new_parameters.len(), 0);
    assert_eq!(ev.changed_parameters.len(), 2);
    assert_eq!(ev.deleted_parameters.len(), 0);
}

trait EventRecordExt {
    fn is_empty(&self) -> bool;
}

impl EventRecordExt for EventRecord {
    fn is_empty(&self) -> bool {
        self.new_parameters.is_empty()
            && self.changed_parameters.is_empty()
            && self.deleted_parameters.is_empty()
    }
}
