//! Action core semantics (transport-neutral).
//!
//! This module is a semantic *validator* and goal state machine.
//! It intentionally does **not** implement ROS transport, QoS, executors,
//! lifecycle gating, timers, or discovery.
//!
//! Authority: `docs/spec/core/action_core.md` (Repo A).
//!
//! # Boundary
//! - Enforces: goal registration, transition validity, terminal immutability,
//!   deterministic decision envelopes, monotonic sequences.
//! - Does not encode: supersession meaning, retention timing, lifecycle policy,
//!   executor behaviour, QoS defaults.
//!
//! Adapters are expected to map these outcomes onto ROS actions
//! (topics/services) and tool-facing behaviour.

mod engine;
mod error;
mod events;
mod policy;
mod state;
mod transition;

pub use engine::{ActionCore, ResultState};
pub use error::ActionError;
pub use events::{FeedbackEvent, FeedbackSequence, ResultAvailable, StatusEvent, StatusSequence};
pub use policy::*;
pub use state::*;
