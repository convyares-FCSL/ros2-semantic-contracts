use thiserror::Error;

#[derive(Debug, Error)]
pub enum CoreError {
    #[error("backend failed with exit code {0}")]
    BackendExit(i32),

    #[error("trace invalid at line {line}: {reason}")]
    Trace { line: usize, reason: String },

    #[error("unsupported expectation: {0}")]
    UnsupportedExpect(String),

    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}
