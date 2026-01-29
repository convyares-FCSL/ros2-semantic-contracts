//! Lifecycle core semantics.
//!
//! This module implements the **core lifecycle state machine** from
//! `docs/spec/core/lifecycle_core.md`.
//!
//! It intentionally does **not** model:
//! - ROS graph APIs or services
//! - executors / threading
//! - resource gating (publishers/timers/etc.)
//!
//! Those belong in global/system/ecosystem layers and harness validation.

mod engine;
mod machine;
mod state;
mod transition;

pub use engine::{
    available_transitions, begin, finish, finish_with_error_handling, goal_state_for_transition,
};
pub use machine::{LifecycleMachine, TransitionInFlight};
pub use state::State;
pub use transition::{CallbackResult, Transition};
