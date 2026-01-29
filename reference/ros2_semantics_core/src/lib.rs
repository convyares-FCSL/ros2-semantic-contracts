//! ros2_semantics_core
//!
//! Reference executable semantics for ROS 2 semantic contracts.
//!
//! This crate provides **pure, transport-agnostic logic** implementing core semantic invariants
//! defined by this repository’s specifications (see `docs/spec/**`).
//!
//! It intentionally contains **no** ROS graph, middleware, DDS/RMW, executor, or scheduling code.
//! Integration layers may expose these semantics via client libraries, but **the specs are
//! authoritative**; this crate is a reference implementation.
//!
//! Current focus:
//! - `parameters`: semantic store + atomic update model + change records.
//! - `lifecycle`: core lifecycle state-machine invariants.
//! - `action`: core action goal-state invariants.

pub mod action;
pub mod error;
pub mod lifecycle;
pub mod parameters;
pub mod support;
