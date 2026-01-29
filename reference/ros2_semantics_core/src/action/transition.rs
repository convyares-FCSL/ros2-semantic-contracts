use super::GoalState;

/// Transition validation rules for *status emissions*.
///
/// Notes:
/// - Re-emission of the same state is allowed.
/// - Some terminal transitions are driven via `set_terminal_result`, not `emit_status`.
pub(crate) fn validate_status_transition(from: GoalState, to: GoalState) -> bool {
    if from == to {
        return true;
    }

    match (from, to) {
        (GoalState::StatusAccepted, GoalState::StatusExecuting) => true,
        (GoalState::StatusExecuting, GoalState::StatusCanceling) => true,

        // Do not allow jumping directly to terminal via emit_status.
        // Terminal resolution is done via `set_terminal_result`.
        (_, t) if t.is_terminal() => false,

        _ => false,
    }
}

/// Terminal completion validation.
///
/// Core allows multiple terminal outcomes under cancellation races,
/// but only if permitted by policy (validated in engine).
pub(crate) fn validate_terminal_transition(from: GoalState, to: GoalState) -> bool {
    if !to.is_terminal() {
        return false;
    }

    match from {
        GoalState::StatusAccepted | GoalState::StatusExecuting | GoalState::StatusCanceling => true,
        _ => false,
    }
}
