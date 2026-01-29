/*
# Support Contract Suites

This file is a black-box contract suite for `ros2_semantics_core::support::*` utilities.

These utilities are NOT core semantic engines and must not leak policy or ROS-facing claims.
They exist to support reference implementations and harness/oracle tooling.

Currently covered:
- `support::gate::ActivationGate`
*/

use ros2_semantics_core::support::gate::ActivationGate;
use std::sync::atomic::{AtomicUsize, Ordering};

//
// -----------------------------------------------------------------------------
// ActivationGate contracts
// -----------------------------------------------------------------------------

#[test]
fn gate_default_is_inactive() {
    let gate = ActivationGate::new();
    assert!(!gate.is_active());
    assert!(gate.allow(|| 1).is_none());
}

#[test]
fn gate_activate_allows_emission() {
    let gate = ActivationGate::new();
    gate.activate();

    assert!(gate.is_active());
    assert_eq!(gate.allow(|| 7), Some(7));
}

#[test]
fn gate_deactivate_suppresses_emission() {
    let gate = ActivationGate::new();
    gate.activate();
    assert_eq!(gate.allow(|| 1), Some(1));

    gate.deactivate();
    assert!(!gate.is_active());
    assert!(gate.allow(|| 2).is_none());
}

#[test]
fn gate_suppressed_emission_has_no_side_effects() {
    let gate = ActivationGate::new(); // inactive
    let hits = AtomicUsize::new(0);

    let out = gate.allow(|| {
        hits.fetch_add(1, Ordering::SeqCst);
        123
    });

    assert!(out.is_none());
    assert_eq!(hits.load(Ordering::SeqCst), 0);
}

#[test]
fn gate_no_queue_or_replay() {
    let gate = ActivationGate::new();
    let hits = AtomicUsize::new(0);

    // Inactive: suppressed, no side effect.
    assert!(gate.allow(|| hits.fetch_add(1, Ordering::SeqCst)).is_none());
    assert_eq!(hits.load(Ordering::SeqCst), 0);

    // Activate: only *new* emissions run; nothing is replayed.
    gate.activate();
    assert!(gate.allow(|| hits.fetch_add(1, Ordering::SeqCst)).is_some());
    assert_eq!(hits.load(Ordering::SeqCst), 1);
}

#[test]
fn gate_repeated_toggles_are_deterministic() {
    let gate = ActivationGate::new();

    gate.activate();
    assert!(gate.is_active());

    gate.deactivate();
    assert!(!gate.is_active());

    gate.activate();
    assert!(gate.is_active());
}
