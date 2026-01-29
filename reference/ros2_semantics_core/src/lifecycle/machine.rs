//! A small state holder around the pure engine functions.
//!
//! This exists for ergonomics and testing; it does not introduce extra semantics.

use crate::error::{CoreError, Domain, ErrorKind, Result};

use super::engine;
use super::state::State;
use super::transition::{CallbackResult, Transition};

#[derive(Debug, Clone)]
pub struct TransitionInFlight {
    pub from: State,
    pub via: Transition,
    pub intermediate: State,
}

#[derive(Debug)]
pub struct LifecycleMachine {
    state: State,
    in_flight: Option<TransitionInFlight>,
}

impl LifecycleMachine {
    pub fn new() -> Self {
        Self {
            state: State::Unconfigured,
            in_flight: None,
        }
    }

    pub fn state(&self) -> State {
        match &self.in_flight {
            Some(f) => f.intermediate,
            None => self.state,
        }
    }

    pub fn stable_state(&self) -> State {
        self.state
    }

    pub fn begin(&mut self, t: Transition) -> Result<State> {
        if self.in_flight.is_some() {
            return Err(CoreError::warn()
                .domain(Domain::Lifecycle)
                .kind(ErrorKind::InvalidState)
                .msg("lifecycle transition rejected: busy (already transitioning)")
                .payload(crate::error::Payload::LifecycleTransition {
                    from_state: self.state as u8,
                    via_transition: t as u8,
                })
                .build());
        }

        let intermediate = engine::begin(self.state, t)?;
        self.in_flight = Some(TransitionInFlight {
            from: self.state,
            via: t,
            intermediate,
        });
        Ok(intermediate)
    }

    pub fn finish(&mut self, cb: CallbackResult) -> Result<State> {
        let flight = self.in_flight.take().ok_or_else(|| {
            CoreError::warn()
                .domain(Domain::Lifecycle)
                .kind(ErrorKind::InvalidState)
                .msg("finish called with no in-flight transition")
                .build()
        })?;

        let next = engine::finish(flight.intermediate, flight.via, cb)?;
        match next {
            State::ErrorProcessing => {
                // Remain in ErrorProcessing until error handling completes.
                self.state = State::ErrorProcessing;
                Ok(State::ErrorProcessing)
            }
            stable => {
                self.state = stable;
                Ok(stable)
            }
        }
    }

    pub fn finish_error_processing(&mut self, cb: CallbackResult) -> Result<State> {
        if self.state != State::ErrorProcessing {
            return Err(CoreError::warn()
                .domain(Domain::Lifecycle)
                .kind(ErrorKind::InvalidState)
                .msg("finish_error_processing called when not in ErrorProcessing")
                .build());
        }

        let stable = engine::finish_with_error_handling(cb);
        self.state = stable;
        Ok(stable)
    }

    pub fn available_transitions(&self) -> Vec<Transition> {
        engine::available_transitions(self.state)
    }
}
