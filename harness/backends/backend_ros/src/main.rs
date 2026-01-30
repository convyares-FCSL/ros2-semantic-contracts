//! ROS 2 Backend for the Oracle Harness.
//!
//! Emits append-only JSONL trace events (observations only).
//!
//! # Contract
//! - CLI: backend_ros <scenario_bundle.json> <trace.jsonl>
//! - Output: JSONL events (one object per line), append-only
//! - Exit 0: Successful execution
//! - Exit 2: Usage/config error
//! - Exit 4: System/internal error

mod bundle;
mod error;
mod io;
mod ops;
mod state;

use clap::Parser;
use rclrs::CreateBasicExecutor;
use serde_json::json;

use crate::{
    bundle::read_bundle,
    error::{BackendError, ExitCode},
    io::EventWriter,
    ops::run_scenario,
};

/// CLI arguments.
#[derive(Parser, Debug)]
#[command(name = "backend_ros", version)]
struct Args {
    /// Input bundle path (JSON).
    scenario_bundle: std::path::PathBuf,
    /// Output trace path (JSONL, append-only).
    trace_jsonl: std::path::PathBuf,
}

fn main() {
    let args = Args::parse();

    match run(args) {
        Ok(()) => std::process::exit(ExitCode::Ok as i32),
        Err(e) => {
            eprintln!("{}", e.render());
            std::process::exit(e.exit_code() as i32);
        }
    }
}

fn run(args: Args) -> Result<(), BackendError> {
    // ---- Boot ROS (rclrs 0.7 style) ----
    let ros_ctx = rclrs::Context::default_from_env()
        .map_err(|e| BackendError::system(e).context("init rclrs Context"))?;
    let exec = ros_ctx.create_basic_executor();
    let _node = exec
        .create_node("oracle_backend_ros")
        .map_err(|e| BackendError::system(e).context("create rclrs node"))?;

    // ---- Parse bundle ----
    let bundle = read_bundle(&args.scenario_bundle)?;

    // ---- Trace ----
    let mut w = EventWriter::new(&args.trace_jsonl)?;

    w.emit(json!({
        "type": "run_start",
        "scenario_id": "_run",
        "detail": {
            "backend": "ros",
            "ros_distro": std::env::var("ROS_DISTRO").unwrap_or_else(|_| "unknown".to_string()),
            "rmw_implementation": std::env::var("RMW_IMPLEMENTATION").unwrap_or_else(|_| "default".to_string())
        }
    }))?;

    w.emit(json!({
        "type": "backend_capabilities",
        "scenario_id": "_run",
        "detail": {
            "backend": "ros",
            "caps": [
                "actions.basic",
                "actions.terminal",
                "ros.params.set"
            ],
            "limits": {}
        }
    }))?;

    let _node_clone = _node.clone();
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let running_clone = running.clone();

    // Spawn spinner thread
    let spinner_handle = std::thread::spawn(move || {
        let mut exec = exec;
        while running_clone.load(std::sync::atomic::Ordering::Relaxed) {
            let opts = rclrs::SpinOptions::default().timeout(std::time::Duration::from_millis(100));
            for err in exec.spin(opts) {
                if let rclrs::RclrsError::RclError {
                    code: rclrs::RclReturnCode::Timeout,
                    ..
                } = err
                {
                    continue;
                }
                eprintln!("Spin warn: {}", err);
            }
        }
    });

    // Create client for self-parameter setting
    let param_client = _node
        .create_client::<rclrs::vendor::rcl_interfaces::srv::SetParameters>(
            "/oracle_backend_ros/set_parameters",
        )
        .map_err(|e| BackendError::system(e).context("create set_parameters client"))?;

    for (scenario_id, scenario) in &bundle.scenarios {
        run_scenario(&mut w, scenario_id, scenario, &param_client)?;
    }

    w.emit(json!({
        "type": "run_end",
        "scenario_id": "_run",
        "detail": { "backend": "ros" }
    }))?;

    // Signal thread to stop
    running.store(false, std::sync::atomic::Ordering::Relaxed);
    let _ = spinner_handle.join();

    w.flush()?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;
    use std::fs;
    use std::path::{Path, PathBuf};

    fn tmp_path(prefix: &str, ext: &str) -> PathBuf {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        std::env::temp_dir().join(format!("{prefix}_{now}.{ext}"))
    }

    fn read_jsonl(path: &Path) -> Vec<Value> {
        let s = fs::read_to_string(path).expect("read jsonl");
        s.lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| serde_json::from_str::<Value>(l).expect("parse jsonl line"))
            .collect()
    }

    fn assert_required_headers(ev: &Value) {
        let obj = ev.as_object().expect("event is object");
        for k in [
            "version",
            "run_id",
            "t_ns",
            "sequence",
            "type",
            "scenario_id",
        ] {
            assert!(obj.contains_key(k), "missing header: {k}");
        }
    }

    fn assert_no_forbidden_judgement_keys(ev: &Value) {
        let obj = ev.as_object().expect("event is object");
        for k in ["ok", "pass", "fail", "verdict", "assertion"] {
            assert!(
                !obj.contains_key(k),
                "forbidden judgement key present in backend event: {k}"
            );
        }
    }

    #[test]
    fn event_writer_enriches_headers_and_increments_sequence() {
        let trace = tmp_path("oracle_backend_ros_trace", "jsonl");
        let _ = fs::remove_file(&trace);

        std::env::set_var("ORACLE_VERSION", "0.1_test");
        std::env::set_var("ORACLE_RUN_ID", "run_test");

        let mut w = EventWriter::new(&trace).expect("EventWriter::new");

        w.emit(json!({
            "type": "run_start",
            "scenario_id": "_run",
            "detail": { "backend": "ros" }
        }))
        .expect("emit run_start");

        w.emit(json!({
            "type": "run_end",
            "scenario_id": "_run",
            "detail": { "backend": "ros" }
        }))
        .expect("emit run_end");

        w.flush().expect("flush");

        let events = read_jsonl(&trace);
        assert_eq!(events.len(), 2);

        for ev in &events {
            assert_required_headers(ev);
            assert_no_forbidden_judgement_keys(ev);
        }

        assert_eq!(events[0]["version"], json!("0.1_test"));
        assert_eq!(events[0]["run_id"], json!("run_test"));

        // sequence must be contiguous
        assert_eq!(events[0]["sequence"], json!(0));
        assert_eq!(events[1]["sequence"], json!(1));

        // t_ns should be monotonic (non-decreasing)
        let t0 = events[0]["t_ns"].as_u64().unwrap();
        let t1 = events[1]["t_ns"].as_u64().unwrap();
        assert!(t1 >= t0);

        let _ = fs::remove_file(&trace);
    }

    #[test]
    fn read_bundle_parses_btreemap_scenarios() {
        let bundle_path = tmp_path("oracle_backend_ros_bundle", "json");
        let bundle_json = json!({
            "version": "0.1",
            "group": "A",
            "scenarios": {
                "A01_unique_goal_identity": { "ops": [], "expects": [] },
                "A02_terminal_immutable":   { "ops": [], "expects": [] }
            }
        });
        fs::write(
            &bundle_path,
            serde_json::to_string_pretty(&bundle_json).unwrap(),
        )
        .expect("write bundle");

        let bundle = read_bundle(&bundle_path).expect("read_bundle");
        assert!(bundle.scenarios.contains_key("A01_unique_goal_identity"));
        assert!(bundle.scenarios.contains_key("A02_terminal_immutable"));

        let _ = fs::remove_file(&bundle_path);
    }

    #[test]
    fn skeleton_run_emits_expected_event_types_and_scenario_ids() {
        let bundle_path = tmp_path("oracle_backend_ros_bundle", "json");
        let trace_path = tmp_path("oracle_backend_ros_trace", "jsonl");

        let bundle_json = json!({
            "version": "0.1",
            "group": "A",
            "scenarios": {
                "A01_unique_goal_identity": { "ops": [] },
                "A02_terminal_immutable":   { "ops": [] }
            }
        });
        fs::write(
            &bundle_path,
            serde_json::to_string_pretty(&bundle_json).unwrap(),
        )
        .expect("write bundle");

        let bundle = read_bundle(&bundle_path).expect("read bundle");
        let mut w = EventWriter::new(&trace_path).expect("EventWriter::new");

        // Mimic current backend_ros main loop (without ROS init)
        w.emit(json!({
            "type": "run_start",
            "scenario_id": "_run",
            "detail": { "backend": "ros" }
        }))
        .unwrap();

        for (scenario_id, scenario) in &bundle.scenarios {
            std::hint::black_box(&scenario.ops);

            w.emit(json!({
                "type": "scenario_start",
                "scenario_id": scenario_id,
                "detail": { "backend": "ros" }
            }))
            .unwrap();

            w.emit(json!({
                "type": "scenario_end",
                "scenario_id": scenario_id,
                "detail": { "backend": "ros", "note": "test scaffold" }
            }))
            .unwrap();
        }

        w.emit(json!({
            "type": "run_end",
            "scenario_id": "_run",
            "detail": { "backend": "ros" }
        }))
        .unwrap();

        w.flush().unwrap();

        let events = read_jsonl(&trace_path);

        // 1 run_start + (2 scenarios * 2) + 1 run_end = 6
        assert_eq!(events.len(), 6);

        for ev in &events {
            assert_required_headers(ev);
            assert_no_forbidden_judgement_keys(ev);
        }

        // First/last types
        assert_eq!(events[0]["type"], json!("run_start"));
        assert_eq!(events[0]["scenario_id"], json!("_run"));
        assert_eq!(events[5]["type"], json!("run_end"));
        assert_eq!(events[5]["scenario_id"], json!("_run"));

        // Scenario IDs must appear for start/end pairs
        let mut seen = std::collections::BTreeMap::<String, (u32, u32)>::new();
        for ev in &events {
            let sid = ev["scenario_id"].as_str().unwrap().to_string();
            let ty = ev["type"].as_str().unwrap();
            if sid == "_run" {
                continue;
            }
            let entry = seen.entry(sid).or_insert((0, 0));
            match ty {
                "scenario_start" => entry.0 += 1,
                "scenario_end" => entry.1 += 1,
                _ => {}
            }
        }

        assert_eq!(seen.len(), 2);
        for (_sid, (starts, ends)) in seen {
            assert_eq!(starts, 1);
            assert_eq!(ends, 1);
        }

        // sequence must be contiguous from 0..N-1
        for (i, ev) in events.iter().enumerate() {
            assert_eq!(ev["sequence"], json!(i as u64));
        }

        let _ = fs::remove_file(&bundle_path);
        let _ = fs::remove_file(&trace_path);
    }
}
