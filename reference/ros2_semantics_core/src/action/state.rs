use core::fmt;

/// Stable identifier for a goal.
///
/// This is intentionally transport-neutral. Adapters map to ROS goal IDs.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct GoalId([u8; 16]);

impl GoalId {
    pub const fn new(bytes: [u8; 16]) -> Self {
        Self(bytes)
    }

    pub const fn bytes(&self) -> &[u8; 16] {
        &self.0
    }
}

impl fmt::Display for GoalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Short, deterministic display for logs/tests.
        write!(f, "{:02x?}", &self.0[..4])
    }
}

/// Core goal status states.
///
/// This mirrors the semantic envelope of ROS actions without redefining ROS IDL.
/// Meaning (supersession, tooling interpretation, etc.) is outside core.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum GoalState {
    StatusAccepted,
    StatusExecuting,
    StatusCanceling,

    StatusSucceeded,
    StatusAborted,
    StatusCanceled,

    /// Reserved only for *query outcomes* when a goal is unknown.
    /// Core MUST NOT persist this as a state for a known goal.
    StatusUnknown,
}

impl GoalState {
    pub const fn is_terminal(self) -> bool {
        matches!(
            self,
            Self::StatusSucceeded | Self::StatusAborted | Self::StatusCanceled
        )
    }
}
