//! Pure lifecycle transition logic (no storage, no threading).

use crate::error::{CoreError, Domain, ErrorKind, Result};

use super::state::State;
use super::transition::{CallbackResult, Transition};

/// Returns the stable goal state implied by a transition when it completes successfully.
pub fn goal_state_for_transition(t: Transition) -> State {
    match t {
        Transition::Configure => State::Inactive,
        Transition::Cleanup => State::Unconfigured,
        Transition::Activate => State::Active,
        Transition::Deactivate => State::Inactive,
        Transition::Shutdown => State::Finalized,
    }
}

/// Begin a transition from a **primary** state.
///
/// Core rule: **no transitions may start from a transitional state**.
/// That includes Shutdown: shutdown-while-transitioning is a system/ecosystem policy,
/// not core lifecycle truth.
pub fn begin(from: State, t: Transition) -> Result<State> {
    // Core: transitional states cannot accept new transitions.
    if !from.is_primary() {
        return Err(CoreError::warn()
            .domain(Domain::Lifecycle)
            .kind(ErrorKind::InvalidState)
            .msg("lifecycle transition rejected: busy (already transitioning)")
            .payload(crate::error::Payload::LifecycleTransition {
                from_state: from as u8,
                via_transition: t as u8,
            })
            .build());
    }

    // Finalized cannot transition.
    if from == State::Finalized {
        return Err(CoreError::invalid_transition_lifecycle(from as u8, t as u8));
    }

    // Canonical graph.
    let allowed = match (from, t) {
        (State::Unconfigured, Transition::Configure) => true,
        (State::Inactive, Transition::Cleanup) => true,
        (State::Inactive, Transition::Activate) => true,
        (State::Active, Transition::Deactivate) => true,
        // Shutdown is allowed from any non-finalized *primary* state.
        (State::Unconfigured, Transition::Shutdown) => true,
        (State::Inactive, Transition::Shutdown) => true,
        (State::Active, Transition::Shutdown) => true,
        _ => false,
    };

    if !allowed {
        return Err(CoreError::invalid_transition_lifecycle(from as u8, t as u8));
    }

    Ok(t.intermediate_state())
}

/// Finish a transition from its intermediate state.
///
/// If callback returns Failure or Error, the caller should route through error handling.
pub fn finish(intermediate: State, via: Transition, cb: CallbackResult) -> Result<State> {
    // Verify intermediate matches the transition.
    if intermediate != via.intermediate_state() {
        return Err(CoreError::warn()
            .domain(Domain::Lifecycle)
            .kind(ErrorKind::InvalidArgument)
            .msg("finish called with mismatched intermediate state")
            .payload(crate::error::Payload::LifecycleTransition {
                from_state: intermediate as u8,
                via_transition: via as u8,
            })
            .build());
    }

    match cb {
        CallbackResult::Success => Ok(goal_state_for_transition(via)),
        CallbackResult::Failure | CallbackResult::Error => Ok(State::ErrorProcessing),
    }
}

/// Finish via error handling.
///
/// Note: mapping from ErrorProcessing -> stable state is **baseline profile policy**
/// and may be marked UNVALIDATED in specs until oracle confirms Jazzy+rclcpp behaviour.
///
/// This reference engine implements the repository’s baseline profile mapping:
/// - Success -> Unconfigured
/// - Failure/Error -> Finalized
pub fn finish_with_error_handling(cb: CallbackResult) -> State {
    match cb {
        CallbackResult::Success => State::Unconfigured,
        CallbackResult::Failure | CallbackResult::Error => State::Finalized,
    }
}

/// Available transitions from a state (primary only).
pub fn available_transitions(from: State) -> Vec<Transition> {
    if !from.is_primary() {
        return Vec::new();
    }

    match from {
        State::Unconfigured => vec![Transition::Configure, Transition::Shutdown],
        State::Inactive => vec![
            Transition::Cleanup,
            Transition::Activate,
            Transition::Shutdown,
        ],
        State::Active => vec![Transition::Deactivate, Transition::Shutdown],
        State::Finalized => vec![],
        _ => vec![],
    }
}
