//! Lifecycle transition identifiers and callback outcomes.

use super::state::State;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[repr(u8)]
pub enum Transition {
    Configure = 1,
    Cleanup = 2,
    Activate = 3,
    Deactivate = 4,
    Shutdown = 5,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum CallbackResult {
    Success,
    Failure,
    Error,
}

impl Transition {
    /// Returns the expected transitional state when this transition begins.
    pub fn intermediate_state(self) -> State {
        match self {
            Transition::Configure => State::Configuring,
            Transition::Cleanup => State::CleaningUp,
            Transition::Activate => State::Activating,
            Transition::Deactivate => State::Deactivating,
            Transition::Shutdown => State::ShuttingDown,
        }
    }
}
