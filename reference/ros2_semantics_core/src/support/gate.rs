//! support::gate
//!
//! Activation-gated emission helper.
//!
//! This module provides a small, explicit utility for suppressing outputs
//! while a component is inactive. It is intended for use by reference
//! implementations and oracle harnesses to model ecosystem-level behaviour
//! (e.g. lifecycle-managed publisher gating in rclcpp).
//!
//! IMPORTANT:
//! - This is NOT lifecycle core semantics.
//! - This does NOT model timers, subscriptions, or actions.
//! - This does NOT imply that lifecycle == gating.
//!
//! Specs:
//! - docs/spec/lifecycle.md (observable rule, UNVALIDATED until oracle)
//! - NOT referenced by docs/spec/core/lifecycle_core.md

use std::sync::atomic::{AtomicBool, Ordering};

/// A simple activation gate.
///
/// When inactive, gated emissions are suppressed.
/// When active, gated emissions are allowed.
///
/// This models *observable behaviour only*.
/// It does not queue, delay, or replay suppressed emissions.
#[derive(Debug, Default)]
pub struct ActivationGate {
    active: AtomicBool,
}

impl ActivationGate {
    /// Create a new gate.
    ///
    /// Gates are inactive by default.
    pub fn new() -> Self {
        Self {
            active: AtomicBool::new(false),
        }
    }

    /// Mark the gate as active.
    ///
    /// After this call, emissions guarded by [`allow`] will be permitted.
    pub fn activate(&self) {
        self.active.store(true, Ordering::Release);
    }

    /// Mark the gate as inactive.
    ///
    /// After this call, emissions guarded by [`allow`] will be suppressed.
    pub fn deactivate(&self) {
        self.active.store(false, Ordering::Release);
    }

    /// Returns whether emissions are currently permitted.
    ///
    /// This is an *observable state*, suitable for testing.
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Acquire)
    }

    /// Execute `emit` only if the gate is active.
    ///
    /// Returns:
    /// - `Some(T)` if the gate is active and `emit` was executed
    /// - `None` if the gate is inactive and emission was suppressed
    ///
    /// No side effects occur when inactive.
    pub fn allow<T>(&self, emit: impl FnOnce() -> T) -> Option<T> {
        if self.is_active() {
            Some(emit())
        } else {
            None
        }
    }
}
