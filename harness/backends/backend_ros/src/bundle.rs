use crate::error::BackendError;
use serde::Deserialize;
use serde_json::Value;
use std::{collections::BTreeMap, fs::File, io::BufReader, path::Path};

#[derive(Deserialize, Debug)]
pub struct ScenarioBundle {
    pub scenarios: BTreeMap<String, Scenario>,
}

#[derive(Deserialize, Debug)]
pub struct Scenario {
    #[serde(default)]
    pub ops: Vec<Op>,
}

#[derive(Deserialize, Debug)]
pub struct Op {
    pub op: String,
    #[serde(default)]
    pub goal_id: Option<String>,
    #[serde(default)]
    pub ms: Option<u64>,
    #[serde(default)]
    pub status: Option<String>,
    #[serde(default)]
    pub payload: Option<Value>,
}

pub fn read_bundle(path: &Path) -> Result<ScenarioBundle, BackendError> {
    let f = File::open(path)
        .map_err(|e| BackendError::system(e).context(format!("read bundle {:?}", path)))?;
    let reader = BufReader::new(f);

    serde_json::from_reader(reader)
        .map_err(|e| BackendError::usage(format!("parse bundle JSON: {e}")))
}
