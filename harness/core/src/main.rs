//! Oracle Core Runner.
//!
//! Core is the deterministic “Judge” of the pipeline.
//! It orchestrates backend execution, validates the trace, evaluates expectations,
//! writes a report, and prints a human summary.

mod error;
mod eval;
mod io;
mod model;
mod report;

use anyhow::{Context, Result};
use clap::Parser;

use crate::error::CoreError;
use crate::eval::evaluate_bundle;
use crate::io::{read_and_validate_trace, read_bundle, run_backend};
use crate::model::Args;
use crate::report::print_summary;

fn main() {
    let args = Args::parse();

    match run(args) {
        Ok(has_fail) => std::process::exit(if has_fail { 1 } else { 0 }),
        Err(e) => {
            eprintln!("Error: {:#}", e);
            match e.downcast_ref::<CoreError>() {
                Some(CoreError::BackendExit(_)) => std::process::exit(4),
                Some(CoreError::Trace { .. }) => std::process::exit(2),
                _ => std::process::exit(4),
            }
        }
    }
}

/// Returns Ok(true) if any scenario FAILED, Ok(false) if no FAIL (PASS + SKIP only),
/// Err if System/Configuration error.
fn run(args: Args) -> Result<bool> {
    let bundle = read_bundle(&args.scenario_bundle)?;

    // 1) Prepare workspace
    std::fs::write(&args.trace_jsonl, "")
        .with_context(|| format!("truncate trace {:?}", args.trace_jsonl))?;

    // 2) Invoke backend
    let backend_code = run_backend(&args.backend, &args.scenario_bundle, &args.trace_jsonl)?;
    if backend_code != 0 {
        return Err(CoreError::BackendExit(backend_code).into());
    }

    // 3) Load & validate trace
    let events = read_and_validate_trace(&args.trace_jsonl)?;

    // 4) Evaluate + write report
    let report = evaluate_bundle(&bundle, events)?;
    std::fs::write(&args.report_json, serde_json::to_string_pretty(&report)?)
        .with_context(|| format!("write report {:?}", args.report_json))?;

    // 5) Human summary
    let counts = print_summary(&report);

    // Integrity check: summary should cover all scenarios.
    let expected_total = bundle.scenarios.len();
    let observed_total = counts.pass + counts.fail + counts.skip;
    if observed_total != expected_total {
        return Err(CoreError::Unexpected(anyhow::anyhow!(
            "internal: summary count mismatch (expected {expected_total}, got {observed_total})"
        ))
        .into());
    }

    Ok(counts.fail > 0)
}
