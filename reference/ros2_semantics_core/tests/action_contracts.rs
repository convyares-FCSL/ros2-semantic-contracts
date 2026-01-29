use ros2_semantics_core::action::*;

fn policies(concurrency: ConcurrencyPolicy, cancel: CancelPolicy) -> PolicySet {
    PolicySet {
        duplicate_goal_id: DuplicateGoalIdPolicy::Reject,
        concurrency,
        retention: RetentionPolicy::RetainForCount(1),
        get_result: GetResultMode::ReturnNotReady,
        cancel,
    }
}

fn cancel_allows_all() -> CancelPolicy {
    CancelPolicy {
        allow_canceled: true,
        allow_succeeded: true,
        allow_aborted: true,
    }
}

#[test]
fn no_ghost_goals_after_rejection() {
    let mut core =
        ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::SingleGoalRejectNew, cancel_allows_all()));

    let goal_id1 = GoalId::new([1u8; 16]);
    let goal_id2 = GoalId::new([2u8; 16]);

    let d1 = core.send_goal_decide(goal_id1, ());
    assert!(d1.accepted);

    let d2 = core.send_goal_decide(goal_id2, ());
    assert!(!d2.accepted);
    assert_eq!(d2.reason, Some(SendGoalRejectionReason::ConcurrencyPolicy));

    // Rejected goals are never registered ("no ghosts").
    assert_eq!(
        core.emit_status(&goal_id2, GoalState::StatusAccepted).unwrap_err(),
        ActionError::UnknownGoal
    );
    assert_eq!(core.emit_feedback(&goal_id2, 1).unwrap_err(), ActionError::UnknownGoal);
    assert_eq!(
        core.set_terminal_result(&goal_id2, GoalState::StatusSucceeded, 7).unwrap_err(),
        ActionError::UnknownGoal
    );

    let cancel_unknown = core.apply_cancel(CancelRequest::GoalId(goal_id2));
    assert_eq!(cancel_unknown.decisions.len(), 1);
    assert_eq!(cancel_unknown.decisions[0].goal_id, goal_id2);
    assert!(!cancel_unknown.decisions[0].accepted);
    assert_eq!(
        cancel_unknown.decisions[0].reason,
        Some(CancelRejectionReason::UnknownGoal)
    );

    assert!(matches!(core.query_result(&goal_id2), ResultState::UnknownGoal));
}

#[test]
fn initial_state_is_accepted_and_transitions_are_validated() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));

    let goal_id = GoalId::new([3u8; 16]);
    assert!(core.send_goal_decide(goal_id, ()).accepted);

    let accepted = core.emit_status(&goal_id, GoalState::StatusAccepted).unwrap();
    assert_eq!(accepted.state, GoalState::StatusAccepted);

    let err = core.emit_status(&goal_id, GoalState::StatusCanceling).unwrap_err();
    assert_eq!(
        err,
        ActionError::InvalidTransition {
            from: GoalState::StatusAccepted,
            to: GoalState::StatusCanceling
        }
    );

    let executing = core.emit_status(&goal_id, GoalState::StatusExecuting).unwrap();
    assert_eq!(executing.state, GoalState::StatusExecuting);

    let canceling = core.emit_status(&goal_id, GoalState::StatusCanceling).unwrap();
    assert_eq!(canceling.state, GoalState::StatusCanceling);

    let (_s, result_avail, _terminal) =
        core.set_terminal_result(&goal_id, GoalState::StatusCanceled, 9).unwrap();
    assert_eq!(result_avail.terminal_state, GoalState::StatusCanceled);
    assert_eq!(result_avail.result, 9);
}

#[test]
fn status_sequence_is_strictly_monotonic_even_on_re_emission() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));

    let goal_id = GoalId::new([4u8; 16]);
    assert!(core.send_goal_decide(goal_id, ()).accepted);

    let s1 = core.emit_status(&goal_id, GoalState::StatusAccepted).unwrap();
    let s2 = core.emit_status(&goal_id, GoalState::StatusAccepted).unwrap();

    assert!(s2.sequence.value() > s1.sequence.value());
    assert_eq!(s1.state, GoalState::StatusAccepted);
    assert_eq!(s2.state, GoalState::StatusAccepted);
}

#[test]
fn feedback_sequence_is_strictly_monotonic() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));

    let goal_id = GoalId::new([5u8; 16]);
    assert!(core.send_goal_decide(goal_id, ()).accepted);

    core.emit_status(&goal_id, GoalState::StatusExecuting).unwrap();

    let f1 = core.emit_feedback(&goal_id, 1).unwrap();
    let f2 = core.emit_feedback(&goal_id, 2).unwrap();

    assert!(f2.sequence.value() > f1.sequence.value());
    assert_eq!(f1.goal_id, goal_id);
    assert_eq!(f2.goal_id, goal_id);
}

#[test]
fn terminal_immutability_and_result_stability() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));

    let goal_id = GoalId::new([6u8; 16]);
    assert!(core.send_goal_decide(goal_id, ()).accepted);
    core.emit_status(&goal_id, GoalState::StatusExecuting).unwrap();

    let (_s, result_avail, _terminal) =
        core.set_terminal_result(&goal_id, GoalState::StatusSucceeded, 42).unwrap();
    assert_eq!(result_avail.terminal_state, GoalState::StatusSucceeded);
    assert_eq!(result_avail.result, 42);

    let r1 = core.query_result(&goal_id);
    let r2 = core.query_result(&goal_id);
    assert_eq!(r1, r2);

    assert!(matches!(
        r1,
        ResultState::Terminal {
            status: GoalState::StatusSucceeded,
            result: 42
        }
    ));

    assert_eq!(
        core.emit_status(&goal_id, GoalState::StatusExecuting).unwrap_err(),
        ActionError::TerminalImmutable
    );
    assert_eq!(
        core.emit_feedback(&goal_id, 9).unwrap_err(),
        ActionError::FeedbackNotAllowed
    );

    let resp = core.apply_cancel(CancelRequest::GoalId(goal_id));
    assert_eq!(resp.decisions.len(), 1);
    assert_eq!(resp.decisions[0].goal_id, goal_id);
    assert!(!resp.decisions[0].accepted);
    assert_eq!(
        resp.decisions[0].reason,
        Some(CancelRejectionReason::TerminalState)
    );
}

#[test]
fn cancel_response_cardinality_and_unknown_goal_outcomes() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));

    let unknown = GoalId::new([7u8; 16]);
    let resp_unknown = core.apply_cancel(CancelRequest::GoalId(unknown));
    assert_eq!(resp_unknown.decisions.len(), 1);
    assert_eq!(resp_unknown.decisions[0].goal_id, unknown);
    assert!(!resp_unknown.decisions[0].accepted);
    assert_eq!(
        resp_unknown.decisions[0].reason,
        Some(CancelRejectionReason::UnknownGoal)
    );

    let g1 = GoalId::new([8u8; 16]);
    let g2 = GoalId::new([9u8; 16]);
    assert!(core.send_goal_decide(g1, ()).accepted);
    assert!(core.send_goal_decide(g2, ()).accepted);

    let resp_all = core.apply_cancel(CancelRequest::AllKnownGoals);
    assert_eq!(resp_all.decisions.len(), 2);

    let mut seen = resp_all.decisions.iter().map(|d| d.goal_id).collect::<Vec<_>>();
    seen.sort();
    assert_eq!(seen, vec![g1, g2]);
    assert!(resp_all.decisions.iter().all(|d| d.accepted));
}

#[test]
fn cancel_race_validation_is_only_validation_not_selection() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));
    let goal_id = GoalId::new([10u8; 16]);

    assert!(core.send_goal_decide(goal_id, ()).accepted);
    core.emit_status(&goal_id, GoalState::StatusExecuting).unwrap();

    let resp = core.apply_cancel(CancelRequest::GoalId(goal_id));
    assert!(resp.decisions[0].accepted);
    assert!(matches!(core.query_result(&goal_id), ResultState::NotReady));

    let (_s, result_avail, _terminal) =
        core.set_terminal_result(&goal_id, GoalState::StatusSucceeded, 7).unwrap();
    assert_eq!(result_avail.terminal_state, GoalState::StatusSucceeded);

    let mut core_disallow_succeeded = ActionCore::<(), i32, i32>::new(policies(
        ConcurrencyPolicy::MultiGoal,
        CancelPolicy {
            allow_canceled: true,
            allow_succeeded: false,
            allow_aborted: true,
        },
    ));

    let goal_id2 = GoalId::new([11u8; 16]);
    assert!(core_disallow_succeeded.send_goal_decide(goal_id2, ()).accepted);
    core_disallow_succeeded
        .emit_status(&goal_id2, GoalState::StatusExecuting)
        .unwrap();
    assert!(core_disallow_succeeded
        .apply_cancel(CancelRequest::GoalId(goal_id2))
        .decisions[0]
        .accepted);

    assert!(matches!(
        core_disallow_succeeded
            .set_terminal_result(&goal_id2, GoalState::StatusSucceeded, 1)
            .unwrap_err(),
        ActionError::InvalidTransition {
            from: GoalState::StatusExecuting,
            to: GoalState::StatusSucceeded
        }
    ));
}

#[test]
fn get_result_outcomes_are_deterministic() {
    let mut core = ActionCore::<(), i32, i32>::new(policies(ConcurrencyPolicy::MultiGoal, cancel_allows_all()));

    let unknown = GoalId::new([12u8; 16]);
    assert!(matches!(core.query_result(&unknown), ResultState::UnknownGoal));

    let goal_id = GoalId::new([13u8; 16]);
    assert!(core.send_goal_decide(goal_id, ()).accepted);
    assert!(matches!(core.query_result(&goal_id), ResultState::NotReady));

    core.emit_status(&goal_id, GoalState::StatusExecuting).unwrap();
    core.set_terminal_result(&goal_id, GoalState::StatusAborted, 5).unwrap();

    assert!(matches!(
        core.query_result(&goal_id),
        ResultState::Terminal {
            status: GoalState::StatusAborted,
            result: 5
        }
    ));
}
