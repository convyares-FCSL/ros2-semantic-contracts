//! Stub Backend for the Oracle Harness.
//!
//! A deterministic event emitter used to validate the harness pipeline in isolation.
//!
//! # Architecture
//! - Input: Single-pass execution of a scenario bundle.
//! - Output: Append-only JSONL trace of observations.
//! - State: Ephemeral; no persistence beyond the trace file.
//!
//! # Contract
//! - Exit 0: Successful execution.
//! - Exit 2: Usage/Config error (e.g., malformed bundle, unsupported op).
//! - Exit 4: Internal/System failure (e.g., I/O error).

use anyhow::{anyhow, Context};
use clap::Parser;
use serde::Deserialize;
use serde_json::{json, Value};
use std::{
    collections::{BTreeMap, HashMap},
    fs::{File, OpenOptions},
    io::{BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
use thiserror::Error;

/// CLI arguments.
#[derive(Parser, Debug)]
#[command(name = "backend_stub", version)]
struct Args {
    /// Input bundle path (JSON).
    scenario_bundle: PathBuf,
    /// Output trace path (JSONL, append-only).
    trace_jsonl: PathBuf,
}

/// Root container for grouped scenarios.
///
/// Note: The backend is agnostic to `expects` or `verdicts`; it only executes `ops`.
#[derive(Deserialize, Debug)]
struct ScenarioBundle {
    scenarios: BTreeMap<String, Scenario>,
}

#[derive(Deserialize, Debug)]
struct Scenario {
    ops: Vec<Op>,
}

/// A loose representation of an operation to allow dynamic validation.
///
/// Fields are Optional to defer validation to the execution phase, allowing
/// for precise error reporting on missing fields.
#[derive(Deserialize, Debug)]
struct Op {
    op: String,
    #[serde(default)]
    goal_id: Option<String>,
    #[serde(default)]
    ms: Option<u64>,
    #[serde(default)]
    status: Option<String>,
    #[serde(default)]
    payload: Option<Value>,
}

/// Domain-specific errors mapped to process exit codes.
#[derive(Debug, Error)]
enum BackendError {
    /// Input violation (invalid config, unknown op, schema violation).
    /// Maps to Exit 2.
    #[error("usage error: {0}")]
    Usage(String),

    /// System-level failure (I/O, permissions, OS limits).
    /// Maps to Exit 4.
    #[error(transparent)]
    Unexpected(#[from] anyhow::Error),
}

impl BackendError {
    fn usage(msg: impl Into<String>) -> Self {
        Self::Usage(msg.into())
    }
}

/// Stub-local ephemeral state.
///
/// # Invariants
/// - No attempt is made to mimic ROS execution timing.
/// - The state exists solely to emit observable evidence for invariants.
#[derive(Default)]
struct BackendState {
    /// goal_id -> terminal_status (e.g. "SUCCEEDED")
    terminal: HashMap<String, String>,
    /// goal_id -> terminal set attempts (1 = first terminal set attempt observed)
    terminal_attempts: HashMap<String, u32>,
}

/// Structured trace emitter.
///
/// # Invariants
/// - Output is strictly JSONL (newline-delimited JSON).
/// - Timestamps (`t_ns`) are monotonic relative to process start.
/// - Sequences are contiguous per run.
struct EventWriter {
    writer: BufWriter<File>,
    version: String,
    run_id: String,
    t0: Instant,
    seq: u64,
}

impl EventWriter {
    fn new(trace_path: impl AsRef<Path>) -> Result<Self, BackendError> {
        // Defaults align with the v0.1 spec if environment is unset.
        let version = std::env::var("VERSION").unwrap_or_else(|_| "0.1".to_string());
        let run_id = std::env::var("RUN_ID").unwrap_or_else(|_| "run_local".to_string());

        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(trace_path.as_ref())
            .with_context(|| format!("open trace {:?}", trace_path.as_ref()))?;

        Ok(Self {
            writer: BufWriter::new(file),
            version,
            run_id,
            t0: Instant::now(),
            seq: 0,
        })
    }

    /// Returns elapsed nanoseconds, saturated at u64::MAX to preserve schema limits.
    fn t_ns(&self) -> u64 {
        let ns = self.t0.elapsed().as_nanos();
        ns.min(u64::MAX as u128) as u64
    }

    /// Enriches and writes an event to the stream.
    fn emit(&mut self, mut ev: Value) -> Result<(), BackendError> {
        let obj = ev.as_object_mut().ok_or_else(|| {
            BackendError::Unexpected(anyhow!("internal: event must be an object"))
        })?;

        // Enforce schema metadata consistency.
        obj.entry("version").or_insert(json!(self.version));
        obj.entry("run_id").or_insert(json!(self.run_id));
        obj.entry("t_ns").or_insert(json!(self.t_ns()));
        obj.entry("sequence").or_insert(json!(self.seq));
        self.seq += 1;

        serde_json::to_writer(&mut self.writer, &ev).context("serialize event")?;
        self.writer.write_all(b"\n").context("write newline")?;
        Ok(())
    }
}

fn main() {
    let args = Args::parse();

    // Process boundary: Map Result types to POSIX exit codes.
    match run(args) {
        Ok(()) => std::process::exit(0),
        Err(BackendError::Usage(msg)) => {
            eprintln!("Error: {msg}");
            std::process::exit(2);
        }
        Err(BackendError::Unexpected(e)) => {
            eprintln!("System Failure: {e:#}");
            std::process::exit(4);
        }
    }
}

fn run(args: Args) -> Result<(), BackendError> {
    let bundle = read_bundle(&args.scenario_bundle)?;
    let mut w = EventWriter::new(&args.trace_jsonl)?;

    w.emit(json!({
        "type": "run_start",
        "scenario_id": "_run",
        "detail": { "backend": "stub" }
    }))?;

    // Execution order is determined by BTreeMap keys (alphanumeric sort).
    for (scenario_id, scenario) in &bundle.scenarios {
        run_scenario(&mut w, scenario_id, scenario)?;
    }

    w.emit(json!({ "type": "run_end", "scenario_id": "_run" }))?;

    // Explicit flush required to guarantee tail integrity before exit.
    w.writer.flush().context("flush trace")?;
    Ok(())
}

fn read_bundle(path: &Path) -> Result<ScenarioBundle, BackendError> {
    let f = File::open(path)
        .with_context(|| format!("read bundle {:?}", path))
        .map_err(BackendError::Unexpected)?;

    // Buffered read prevents excessive syscalls during parsing.
    let reader = BufReader::new(f);

    serde_json::from_reader(reader)
        .context("parse bundle JSON")
        .map_err(BackendError::Unexpected)
}

fn run_scenario(w: &mut EventWriter, id: &str, scenario: &Scenario) -> Result<(), BackendError> {
    let mut st = BackendState::default();

    w.emit(json!({
        "type": "scenario_start",
        "scenario_id": id,
        "detail": { "backend": "stub" }
    }))?;

    for (i, op) in scenario.ops.iter().enumerate() {
        let op_id = format!("{id}#{i}");
        exec_op(w, &mut st, id, &op_id, op)?;
    }

    w.emit(json!({ "type": "scenario_end", "scenario_id": id }))?;
    Ok(())
}

fn exec_op(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op_id: &str,
    op: &Op,
) -> Result<(), BackendError> {
    w.emit(json!({
        "type": "op_start",
        "scenario_id": scenario_id,
        "op_id": op_id,
        "detail": { "op": op.op }
    }))?;

    match op.op.as_str() {
        "send_goal" => exec_send_goal(w, scenario_id, op)?,
        "wait" => exec_wait(op)?,
        "complete_terminal" => exec_complete_terminal(w, st, scenario_id, op)?,
        "attempt_terminal_override" => exec_attempt_terminal_override(w, st, scenario_id, op)?,
        other => {
            // Diagnostic event allows trace consumers to see *why* the run failed.
            w.emit(json!({
                "type": "diagnostic",
                "scenario_id": scenario_id,
                "detail": { "error": { "kind": "unsupported_op", "op": other } }
            }))?;
            return Err(BackendError::usage(format!("unsupported op: {other}")));
        }
    }

    w.emit(json!({ "type": "op_end", "scenario_id": scenario_id, "op_id": op_id }))?;
    Ok(())
}

fn exec_send_goal(w: &mut EventWriter, scenario_id: &str, op: &Op) -> Result<(), BackendError> {
    // Validate required fields for this specific operation.
    let goal_id = op
        .goal_id
        .as_deref()
        .ok_or_else(|| BackendError::usage("send_goal missing goal_id"))?;

    let mode = op
        .payload
        .as_ref()
        .and_then(|p| p.get("mode"))
        .and_then(|m| m.as_str())
        .ok_or_else(|| BackendError::usage("send_goal missing payload.mode"))?;

    w.emit(json!({ "type": "goal_send", "scenario_id": scenario_id, "goal_id": goal_id }))?;

    // Simulate backend decision logic.
    let accepted = match mode {
        "accept" => true,
        "reject" => false,
        _ => return Err(BackendError::usage(format!("invalid mode: {mode}"))),
    };

    let mut ev = json!({
        "type": "goal_response",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "accepted": accepted
    });

    if !accepted {
        ev.as_object_mut()
            .unwrap()
            .insert("reason".into(), json!("stub_reject"));
    }

    w.emit(ev)?;
    Ok(())
}

fn exec_complete_terminal(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op: &Op,
) -> Result<(), BackendError> {
    let goal_id = op
        .goal_id
        .as_deref()
        .ok_or_else(|| BackendError::usage("complete_terminal missing goal_id"))?;

    let status = op
        .status
        .as_deref()
        .ok_or_else(|| BackendError::usage("complete_terminal missing status"))?;

    let attempt = st.terminal_attempts.entry(goal_id.to_string()).or_insert(0);
    *attempt += 1;

    // Terminal is immutable: once set, subsequent terminal sets are denied but observable.
    if st.terminal.contains_key(goal_id) {
        w.emit(json!({
            "type": "terminal_set_attempt",
            "scenario_id": scenario_id,
            "goal_id": goal_id,
            "attempt": *attempt,
            "allowed": false,
            "reason": "terminal_immutable"
        }))?;
        return Ok(());
    }

    st.terminal.insert(goal_id.to_string(), status.to_string());

    w.emit(json!({
        "type": "status",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "status": status
    }))?;

    w.emit(json!({
        "type": "result",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "status": status
    }))?;

    Ok(())
}

fn exec_attempt_terminal_override(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op: &Op,
) -> Result<(), BackendError> {
    let goal_id = op
        .goal_id
        .as_deref()
        .ok_or_else(|| BackendError::usage("attempt_terminal_override missing goal_id"))?;

    let _requested = op
        .status
        .as_deref()
        .ok_or_else(|| BackendError::usage("attempt_terminal_override missing status"))?;

    let attempt = st.terminal_attempts.entry(goal_id.to_string()).or_insert(0);
    *attempt += 1;

    // If already terminal, override is denied and observable.
    if st.terminal.contains_key(goal_id) {
        w.emit(json!({
            "type": "terminal_set_attempt",
            "scenario_id": scenario_id,
            "goal_id": goal_id,
            "attempt": *attempt,
            "allowed": false,
            "reason": "terminal_immutable"
        }))?;
        return Ok(());
    }

    // If not terminal yet, record that an override was "allowed" (rare in real systems),
    // but do not fabricate terminal semantics here.
    w.emit(json!({
        "type": "terminal_set_attempt",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "attempt": *attempt,
        "allowed": true
    }))?;

    Ok(())
}

fn exec_wait(op: &Op) -> Result<(), BackendError> {
    let ms = op
        .ms
        .ok_or_else(|| BackendError::usage("wait missing ms"))?;
    std::thread::sleep(Duration::from_millis(ms));
    Ok(())
}
