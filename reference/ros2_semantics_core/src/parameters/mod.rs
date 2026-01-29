// reference/ros2_semantics_core/src/parameters/mod.rs
//! Transport-agnostic parameter store and parameter semantics.
//!
//! This module intentionally contains no ROS services, topics, or transport code.
//! It models the semantic behaviour described by Repo A parameter contracts and produces
//! explicit outcomes plus an event record for adapter emission.

mod store;
mod types;

pub use store::ParameterStore;
pub use types::{
    DescribedParameter, Descriptor, EventRecord, ListResult, Parameter, SetResult, Type, Value,
};
