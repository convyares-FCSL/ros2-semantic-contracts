use super::{GoalId, GoalState};

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct StatusSequence(u64);

impl StatusSequence {
    pub fn new() -> Self {
        Self(0)
    }

    pub fn next(&mut self) -> StatusSequence {
        self.0 = self.0.checked_add(1).expect("status sequence overflow");
        *self
    }

    pub fn value(self) -> u64 {
        self.0
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct FeedbackSequence(u64);

impl FeedbackSequence {
    pub fn new() -> Self {
        Self(0)
    }

    pub fn next(&mut self) -> FeedbackSequence {
        self.0 = self.0.checked_add(1).expect("feedback sequence overflow");
        *self
    }

    pub fn value(self) -> u64 {
        self.0
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct StatusEvent {
    pub goal_id: GoalId,
    pub state: GoalState,
    pub sequence: StatusSequence,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FeedbackEvent<F> {
    pub goal_id: GoalId,
    pub feedback: F,
    pub sequence: FeedbackSequence,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResultAvailable<R> {
    pub terminal_state: GoalState,
    pub result: R,
}
