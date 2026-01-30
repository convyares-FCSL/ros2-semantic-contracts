use std::collections::HashMap;

#[derive(Default)]
pub struct BackendState {
    pub terminal: HashMap<String, String>,
    pub terminal_attempts: HashMap<String, u32>,
}
