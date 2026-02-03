use crate::error::CoreError;
use crate::model::ScenarioBundle;

use anyhow::{Context, Result};
use serde_json::Value;
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
    process::Command,
};

/// Minimal valid core vocabulary.
///
/// This list defines the complete set of event types the oracle will accept
/// from backends. Any new observable evidence must be explicitly added here;
/// unknown types are rejected at trace-ingest time.
const CORE_TYPES: &[&str] = &[
    "run_start",
    "run_end",
    "scenario_start",
    "scenario_end",
    "env",
    "diagnostic",
    "op_start",
    "op_end",
    "goal_send",
    "goal_response",
    "status",
    "result",
    "terminal_set_attempt",
    "backend_capabilities",
    "param_set_request",
    "param_set_response",
    "param_set_batch_request",
    "param_set_batch_response",
    "param_describe_request",
    "param_describe_response",
    "param_declare_request",
    "param_declare_request",
    "param_declare_response",
    "transition_request",
    "transition_response",
    "get_state_request",
    "get_state_response",
    "deadline_exceeded",
    "get_state_error",
];

pub fn read_bundle(path: &Path) -> Result<ScenarioBundle> {
    let f = File::open(path).with_context(|| format!("open bundle {:?}", path))?;
    let reader = BufReader::new(f);
    serde_json::from_reader(reader).context("parse bundle")
}

pub fn run_backend(backend: &str, bundle: &Path, trace: &Path) -> Result<i32> {
    let status = Command::new(backend)
        .arg(bundle)
        .arg(trace)
        .status()
        .with_context(|| format!("execute backend {backend}"))?;

    Ok(status.code().unwrap_or(255))
}

pub fn read_and_validate_trace(path: &Path) -> Result<Vec<Value>> {
    let f = File::open(path).with_context(|| format!("open trace {:?}", path))?;
    let reader = BufReader::new(f);

    let mut out = Vec::new();
    for (idx, line) in reader.lines().enumerate() {
        let line_no = idx + 1;
        let line = line.with_context(|| format!("read trace line {line_no}"))?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let v: Value = serde_json::from_str(trimmed)
            .with_context(|| format!("parse json on line {line_no}"))?;

        if let Err(reason) = validate_event(&v) {
            return Err(CoreError::Trace {
                line: line_no,
                reason,
            }
            .into());
        }

        out.push(v);
    }
    Ok(out)
}

fn validate_event(v: &Value) -> std::result::Result<(), String> {
    let obj = v
        .as_object()
        .ok_or_else(|| "event not an object".to_string())?;

    for k in ["version", "run_id", "t_ns", "type", "scenario_id"] {
        if !obj.contains_key(k) {
            return Err(format!("missing header: {k}"));
        }
    }

    for forbidden in ["ok", "pass", "fail", "verdict", "assertion"] {
        if obj.contains_key(forbidden) {
            return Err(format!("forbidden key: {forbidden} (backend cannot judge)"));
        }
    }

    let ty = obj.get("type").and_then(|x| x.as_str()).unwrap_or("");
    if !CORE_TYPES.contains(&ty) {
        return Err(format!("unknown event type: {ty}"));
    }

    Ok(())
}
