use std::collections::BTreeMap;

use super::events::*;
use super::policy::*;
use super::state::*;
use super::transition::{validate_status_transition, validate_terminal_transition};
use super::ActionError;

/// Result query state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResultState<R> {
    UnknownGoal,
    NotReady,
    Terminal { status: GoalState, result: R },
}

#[derive(Debug)]
struct GoalEntry<R> {
    state: GoalState,
    terminal: Option<(GoalState, R)>,
    status_seq: StatusSequence,
    feedback_seq: FeedbackSequence,

    // Cancel request is a *validation signal*, not a driver.
    cancel_requested: bool,
}

impl<R> GoalEntry<R> {
    fn new() -> Self {
        Self {
            state: GoalState::StatusAccepted,
            terminal: None,
            status_seq: StatusSequence::new(),
            feedback_seq: FeedbackSequence::new(),
            cancel_requested: false,
        }
    }

    fn is_terminal(&self) -> bool {
        self.terminal.is_some()
    }
}

/// ActionCore is a semantic validator + goal state machine.
///
/// It never "chooses outcomes" under races. It validates whether an outcome
/// is permitted by the supplied policies and current recorded intent.
#[derive(Debug)]
pub struct ActionCore<G, R, F> {
    policies: PolicySet,
    goals: BTreeMap<GoalId, GoalEntry<R>>,
    _phantom: core::marker::PhantomData<(G, F)>,
}

impl<G, R: Clone, F> ActionCore<G, R, F> {
    pub fn new(policies: PolicySet) -> Self {
        Self {
            policies,
            goals: BTreeMap::new(),
            _phantom: core::marker::PhantomData,
        }
    }

    /// Evaluate a send_goal request and (if accepted) register the goal.
    ///
    /// Contract: rejected goals MUST NOT be observable ("no ghosts").
    pub fn send_goal_decide(&mut self, goal_id: GoalId, _goal: G) -> SendGoalDecision {
        if self.goals.contains_key(&goal_id) {
            return SendGoalDecision {
                accepted: false,
                reason: Some(SendGoalRejectionReason::DuplicateGoalId),
            };
        }

        if matches!(
            self.policies.concurrency,
            ConcurrencyPolicy::SingleGoalRejectNew
        ) {
            let has_active = self.goals.values().any(|e| !e.is_terminal());
            if has_active {
                return SendGoalDecision {
                    accepted: false,
                    reason: Some(SendGoalRejectionReason::ConcurrencyPolicy),
                };
            }
        }

        self.goals.insert(goal_id, GoalEntry::new());
        SendGoalDecision {
            accepted: true,
            reason: None,
        }
    }

    /// Emit a status state for a known goal.
    ///
    /// Re-emission of the same state is allowed and MUST advance sequence.
    pub fn emit_status(
        &mut self,
        goal_id: &GoalId,
        to: GoalState,
    ) -> Result<StatusEvent, ActionError> {
        let entry = self
            .goals
            .get_mut(goal_id)
            .ok_or(ActionError::UnknownGoal)?;

        if entry.is_terminal() {
            return Err(ActionError::TerminalImmutable);
        }

        if to.is_terminal() {
            return Err(ActionError::InvalidTransition {
                from: entry.state,
                to,
            });
        }

        if !validate_status_transition(entry.state, to) {
            return Err(ActionError::InvalidTransition {
                from: entry.state,
                to,
            });
        }

        entry.state = to;
        let seq = entry.status_seq.next();

        Ok(StatusEvent {
            goal_id: *goal_id,
            state: to,
            sequence: seq,
        })
    }

    /// Emit feedback for a known goal.
    pub fn emit_feedback(
        &mut self,
        goal_id: &GoalId,
        feedback: F,
    ) -> Result<FeedbackEvent<F>, ActionError> {
        let entry = self
            .goals
            .get_mut(goal_id)
            .ok_or(ActionError::UnknownGoal)?;

        if entry.is_terminal() {
            return Err(ActionError::FeedbackNotAllowed);
        }

        // Feedback is only meaningful while executing (core choice).
        if entry.state != GoalState::StatusExecuting {
            return Err(ActionError::FeedbackNotAllowed);
        }

        let seq = entry.feedback_seq.next();
        Ok(FeedbackEvent {
            goal_id: *goal_id,
            feedback,
            sequence: seq,
        })
    }

    /// Apply a cancel request. This validates eligibility and records cancel intent.
    ///
    /// Core does not transition states or complete goals here.
    pub fn apply_cancel(&mut self, req: CancelRequest) -> CancelResponse {
        match req {
            CancelRequest::GoalId(id) => CancelResponse {
                decisions: vec![self.cancel_one(id)],
            },
            CancelRequest::AllKnownGoals => {
                let mut decisions = Vec::with_capacity(self.goals.len());
                for id in self.goals.keys().copied().collect::<Vec<_>>() {
                    decisions.push(self.cancel_one(id));
                }
                CancelResponse { decisions }
            }
        }
    }

    fn cancel_one(&mut self, goal_id: GoalId) -> CancelDecision {
        let Some(entry) = self.goals.get_mut(&goal_id) else {
            return CancelDecision {
                goal_id,
                accepted: false,
                reason: Some(CancelRejectionReason::UnknownGoal),
            };
        };

        if entry.is_terminal() {
            return CancelDecision {
                goal_id,
                accepted: false,
                reason: Some(CancelRejectionReason::TerminalState),
            };
        }

        entry.cancel_requested = true;
        CancelDecision {
            goal_id,
            accepted: true,
            reason: None,
        }
    }

    /// Set the terminal result for a goal.
    ///
    /// This is the only path that commits terminal state.
    pub fn set_terminal_result(
        &mut self,
        goal_id: &GoalId,
        terminal_state: GoalState,
        result: R,
    ) -> Result<(StatusEvent, ResultAvailable<R>, StatusEvent), ActionError> {
        let entry = self
            .goals
            .get_mut(goal_id)
            .ok_or(ActionError::UnknownGoal)?;

        if entry.is_terminal() {
            return Err(ActionError::TerminalImmutable);
        }

        if !validate_terminal_transition(entry.state, terminal_state) {
            return Err(ActionError::InvalidTransition {
                from: entry.state,
                to: terminal_state,
            });
        }

        // Cancel race: validate legality against CancelPolicy.
        // Core validates legality; it does not select the outcome.
        if entry.cancel_requested {
            let allowed = match terminal_state {
                GoalState::StatusCanceled => self.policies.cancel.allow_canceled,
                GoalState::StatusSucceeded => self.policies.cancel.allow_succeeded,
                GoalState::StatusAborted => self.policies.cancel.allow_aborted,
                _ => false,
            };
            if !allowed {
                return Err(ActionError::InvalidTransition {
                    from: entry.state,
                    to: terminal_state,
                });
            }
        }

        entry.terminal = Some((terminal_state, result.clone()));

        // Emit a status event for terminal state (sequence advances).
        entry.state = terminal_state;
        let s1 = StatusEvent {
            goal_id: *goal_id,
            state: terminal_state,
            sequence: entry.status_seq.next(),
        };

        let avail = ResultAvailable {
            terminal_state,
            result,
        };

        // Optional terminal re-emit as a distinct signal (kept for parity with older APIs).
        let s2 = StatusEvent {
            goal_id: *goal_id,
            state: terminal_state,
            sequence: entry.status_seq.next(),
        };

        Ok((s1, avail, s2))
    }

    /// Query the result state.
    pub fn query_result(&self, goal_id: &GoalId) -> ResultState<R> {
        let Some(entry) = self.goals.get(goal_id) else {
            return ResultState::UnknownGoal;
        };

        match &entry.terminal {
            None => ResultState::NotReady,
            Some((st, r)) => ResultState::Terminal {
                status: *st,
                result: r.clone(),
            },
        }
    }
}
