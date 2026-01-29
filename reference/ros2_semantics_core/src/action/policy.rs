use super::GoalId;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DuplicateGoalIdPolicy {
    Reject,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ConcurrencyPolicy {
    /// Only one non-terminal goal may exist; new goals are rejected.
    SingleGoalRejectNew,
    /// Multiple goals may coexist.
    MultiGoal,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RetentionPolicy {
    /// Retain results for a bounded count (core stores, does not evict by time).
    RetainForCount(u64),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GetResultMode {
    /// `get_result` returns `NotReady` if non-terminal.
    ReturnNotReady,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CancelPolicy {
    pub allow_canceled: bool,
    pub allow_succeeded: bool,
    pub allow_aborted: bool,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PolicySet {
    pub duplicate_goal_id: DuplicateGoalIdPolicy,
    pub concurrency: ConcurrencyPolicy,
    pub retention: RetentionPolicy,
    pub get_result: GetResultMode,
    pub cancel: CancelPolicy,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SendGoalRejectionReason {
    DuplicateGoalId,
    ConcurrencyPolicy,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SendGoalDecision {
    pub accepted: bool,
    pub reason: Option<SendGoalRejectionReason>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CancelRejectionReason {
    UnknownGoal,
    TerminalState,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CancelDecision {
    pub goal_id: GoalId,
    pub accepted: bool,
    pub reason: Option<CancelRejectionReason>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CancelResponse {
    pub decisions: Vec<CancelDecision>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CancelRequest {
    GoalId(GoalId),
    AllKnownGoals,
}
