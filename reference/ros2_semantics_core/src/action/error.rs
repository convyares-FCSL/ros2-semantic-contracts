use super::GoalState;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActionError {
    UnknownGoal,
    TerminalImmutable,
    FeedbackNotAllowed,
    InvalidTransition { from: GoalState, to: GoalState },
}
