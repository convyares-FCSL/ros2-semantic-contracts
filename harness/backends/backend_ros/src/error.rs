use std::fmt;

#[derive(Debug, Clone, Copy)]
pub enum ExitCode {
    Ok = 0,
    Usage = 2,
    System = 4,
}

#[derive(Debug)]
pub struct BackendError {
    code: ExitCode,
    msg: String,
    ctx: Vec<String>,
}

impl BackendError {
    pub fn usage(msg: impl Into<String>) -> Self {
        Self {
            code: ExitCode::Usage,
            msg: msg.into(),
            ctx: Vec::new(),
        }
    }

    pub fn system(err: impl fmt::Display) -> Self {
        Self {
            code: ExitCode::System,
            msg: format!("{err}"),
            ctx: Vec::new(),
        }
    }

    pub fn context(mut self, msg: impl Into<String>) -> Self {
        self.ctx.push(msg.into());
        self
    }

    pub fn exit_code(&self) -> ExitCode {
        self.code
    }

    pub fn render(&self) -> String {
        if self.ctx.is_empty() {
            match self.code {
                ExitCode::Usage => format!("Error: {}", self.msg),
                ExitCode::System => format!("System Failure: {}", self.msg),
                ExitCode::Ok => self.msg.clone(),
            }
        } else {
            let mut out = String::new();
            match self.code {
                ExitCode::Usage => out.push_str("Error: "),
                ExitCode::System => out.push_str("System Failure: "),
                ExitCode::Ok => {}
            }
            out.push_str(&self.msg);
            for c in self.ctx.iter().rev() {
                out.push_str("\n  context: ");
                out.push_str(c);
            }
            out
        }
    }
}
