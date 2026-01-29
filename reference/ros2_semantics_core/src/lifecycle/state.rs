//! Lifecycle state identifiers.
//!
//! These are intentionally small/stable and map cleanly onto ROS lifecycle IDs,
//! but this module does not depend on ROS messages.

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(u8)]
pub enum State {
    Unconfigured = 0,
    Inactive = 1,
    Active = 2,
    Finalized = 3,

    // Transitional states (internal only; not required to be observable on ROS graph).
    Configuring = 10,
    CleaningUp = 11,
    Activating = 12,
    Deactivating = 13,
    ShuttingDown = 14,

    // Error handling.
    ErrorProcessing = 99,
}

impl State {
    /// Returns true if this state is a primary (stable) lifecycle state.
    pub fn is_primary(self) -> bool {
        matches!(
            self,
            State::Unconfigured | State::Inactive | State::Active | State::Finalized
        )
    }

    /// Returns true if this state is transitional (in-flight).
    pub fn is_transitional(self) -> bool {
        matches!(
            self,
            State::Configuring
                | State::CleaningUp
                | State::Activating
                | State::Deactivating
                | State::ShuttingDown
                | State::ErrorProcessing
        )
    }
}
