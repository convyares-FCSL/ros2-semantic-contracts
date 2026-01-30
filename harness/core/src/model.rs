use clap::Parser;
use serde::Deserialize;
use serde_json::Value;
use std::{collections::BTreeMap, path::PathBuf};

/// CLI arguments.
#[derive(Parser, Debug)]
#[command(name = "oracle_core", version)]
pub struct Args {
    /// Input scenario bundle.
    pub scenario_bundle: PathBuf,
    /// Shared trace path (Core truncates; Backend appends).
    pub trace_jsonl: PathBuf,
    /// Output report path.
    pub report_json: PathBuf,
    /// Backend executable (must be in PATH or absolute).
    #[arg(long, default_value = "backend_stub")]
    pub backend: String,
}

#[derive(Deserialize, Debug)]
pub struct ScenarioBundle {
    pub version: String,
    pub group: String,
    #[serde(default)]
    pub description: Option<String>,
    pub scenarios: BTreeMap<String, Scenario>,
}

#[derive(Deserialize, Debug)]
pub struct Scenario {
    pub spec_id: String,
    pub title: String,
    #[serde(default)]
    pub layer: Option<String>,
    #[serde(default)]
    pub notes: Option<String>,
    /// Backend instructions (parsed for schema honesty; not interpreted by core).
    #[serde(default)]
    pub ops: Vec<Value>,
    #[serde(default)]
    pub expects: Vec<Expect>,
}

#[derive(Deserialize, Debug)]
pub struct Expect {
    pub check: String,
    #[serde(default)]
    pub params: Option<Value>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    Skip,
}

impl Outcome {
    pub fn as_str(self) -> &'static str {
        match self {
            Outcome::Pass => "PASS",
            Outcome::Fail => "FAIL",
            Outcome::Skip => "SKIP",
        }
    }
}
