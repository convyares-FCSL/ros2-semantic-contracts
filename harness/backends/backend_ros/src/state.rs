use std::collections::{HashMap, HashSet};

#[derive(Default)]
pub struct BackendState {
    pub terminal: HashMap<String, String>,
    pub terminal_attempts: HashMap<String, u32>,
    pub declared_params: HashSet<String>,
}
