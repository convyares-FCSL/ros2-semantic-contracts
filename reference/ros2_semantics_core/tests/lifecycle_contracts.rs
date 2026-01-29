use ros2_semantics_core::lifecycle::{
    available_transitions, begin, finish, CallbackResult, State, Transition,
};

#[test]
fn transitional_states_have_no_available_transitions() {
    let transitional_states = [
        State::Configuring,
        State::CleaningUp,
        State::Activating,
        State::Deactivating,
        State::ShuttingDown,
        State::ErrorProcessing,
    ];

    for state in transitional_states {
        assert!(available_transitions(state).is_empty());
    }
}

#[test]
fn shutdown_from_primary_states_is_supported() {
    for state in [State::Unconfigured, State::Inactive, State::Active] {
        let intermediate = begin(state, Transition::Shutdown).unwrap();
        assert_eq!(intermediate, State::ShuttingDown);
        let final_state =
            finish(intermediate, Transition::Shutdown, CallbackResult::Success).unwrap();
        assert_eq!(final_state, State::Finalized);
    }
}

#[test]
fn shutdown_while_transitioning_is_rejected_as_busy() {
    // Core semantics: no new begin from a transitional state.
    let err = begin(State::Configuring, Transition::Shutdown).unwrap_err();
    assert_eq!(err.domain, ros2_semantics_core::error::Domain::Lifecycle);
    assert_eq!(
        err.kind,
        ros2_semantics_core::error::ErrorKind::InvalidState
    );
}
