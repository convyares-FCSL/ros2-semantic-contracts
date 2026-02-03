#!/usr/bin/env python3
"""
Aggregates probe matrix results into a summary JSON and Markdown table.
"""

import os
import json
import glob
from pathlib import Path
import math

def main():
    evidence_dir = Path("evidence")
    
    distros = ["humble", "jazzy"]
    backends = ["prod", "rclpy"]
    probes = ["P04", "L05", "S11"]
    
    cells = []
    
    # Validation Gate thresholds
    MIN_TRACE_BYTES = 10
    
    for distro in distros:
        for backend in backends:
            for probe in probes:
                cell_id = f"{distro}/{backend}/{probe}"
                cell_dir = evidence_dir / distro / backend / probe
                
                cell_stats = {
                    "runs_total": 0,
                    "runs_passed": 0,
                    "runs_semantic_fail": 0,
                    "runs_infra_fail": 0,
                    "runs_unsupported": 0,
                    "failures": [], # List of dicts with cause/evidence
                    "latencies": []
                }
                
                # Check for run_* directories
                run_dirs = sorted([d for d in cell_dir.glob("run_*") if d.is_dir()])
                
                for run_dir in run_dirs:
                    cell_stats["runs_total"] += 1
                    run_id = run_dir.name
                    report_path = run_dir / "report.json"
                    trace_path = run_dir / "trace.jsonl"
                    stdout_path = run_dir / "stdout.log"
                    
                    # --- Evidence Validity Gate ---
                    gate_passed = True
                    gate_failure_reason = ""
                    
                    if not trace_path.exists():
                        gate_passed = False
                        gate_failure_reason = "Missing trace.jsonl"
                    elif trace_path.stat().st_size < MIN_TRACE_BYTES:
                        gate_passed = False
                        gate_failure_reason = "Empty trace.jsonl"
                        
                    report_data = None
                    if gate_passed:
                        if not report_path.exists():
                            gate_passed = False
                            gate_failure_reason = "Missing report.json"
                        else:
                            try:
                                with open(report_path, 'r') as f:
                                    report_data = json.load(f)
                            except json.JSONDecodeError:
                                gate_passed = False
                                gate_failure_reason = "Invalid JSON in report.json"
                    
                    outcome = "UNKNOWN"
                    if gate_passed and report_data:
                        # Extract outcome from report
                        scenarios = report_data.get("scenarios", {})
                        if not scenarios:
                             gate_passed = False
                             gate_failure_reason = "No scenarios in report"
                        else:
                             # Assume 1 scenario per probe
                             scen_val = list(scenarios.values())[0]
                             outcome = scen_val.get("outcome", "UNKNOWN")
                             if outcome not in ["PASS", "FAIL", "ERROR"]:
                                 # Maybe older format uses verdict?
                                 outcome = scen_val.get("verdict", "UNKNOWN")
                                 
                        # Parse Trace (for S11 latency and validation)
                        latencies = []
                        if trace_path.exists():
                            try:
                                with open(trace_path, 'r') as f:
                                    for line_num, line in enumerate(f, 1):
                                        if not line.strip(): continue
                                        try:
                                            event = json.loads(line)
                                            if event.get("type") == "op_end":
                                                detail = event.get("detail", {})
                                                # STRICT FILTER: Only extract latency from 'get_state' ops
                                                if detail.get("op") == "get_state":
                                                    res = detail.get("result", {})
                                                    if "latency_ms" in res:
                                                        lat = res["latency_ms"]
                                                        if lat >= 0:
                                                            latencies.append(lat)
                                                        else:
                                                            gate_passed = False
                                                            gate_failure_reason = f"Negative latency in S11: {lat}"
                                        except json.JSONDecodeError:
                                            gate_passed = False
                                            gate_failure_reason = f"Invalid JSON in trace line {line_num}"
                                            break
                            except Exception as e:
                                gate_passed = False
                                gate_failure_reason = f"Trace read error: {e}"

                        # Strict Report Validation
                        if report_data:
                            scenarios = report_data.get("scenarios", {})
                            if len(scenarios) != 1:
                                gate_passed = False
                                gate_failure_reason = f"Report has {len(scenarios)} scenarios, expected 1"
                            else:
                                for s_name, s_data in scenarios.items():
                                    if "outcome" not in s_data:
                                        gate_passed = False
                                        gate_failure_reason = f"Scenario {s_name} missing outcome"

                    
                    # --- Classification ---
                    classification = "UNKNOWN"
                    root_cause = ""
                    
                    if not gate_passed:
                        classification = "INFRA_FAIL"
                        root_cause = f"Gate Failed: {gate_failure_reason}"
                        cell_stats["runs_infra_fail"] += 1
                    elif outcome == "PASS":
                        classification = "PASS"
                        cell_stats["runs_passed"] += 1
                    elif outcome == "FAIL":
                        classification = "SEMANTIC_FAIL"
                        root_cause = "Semantic Check Failed" 
                        # Try to get expectation detail
                        try:
                             scen_val = list(report_data["scenarios"].values())[0]
                             violations = scen_val.get("details", {}).get("violations", {})
                             if violations.get("missing_expectations"):
                                 missed = violations["missing_expectations"][0]
                                 root_cause = f"Missing: {missed.get('type')}"
                        except:
                             pass
                        cell_stats["runs_semantic_fail"] += 1
                    else:
                        # ERROR or weird state in report
                        classification = "SEMANTIC_FAIL" # Or INFRA? ERROR usually means Oracle Core crashed or logic error
                        root_cause = f"Harness Error: {outcome}"
                        cell_stats["runs_semantic_fail"] += 1

                    # Handle logical UNSUPPORTED for prod L05/S11 (if explicitly flagged or expected)
                    # For now treating all non-passes as failures unless marked unsupported.
                    
                    if classification != "PASS":
                        cell_stats["failures"].append({
                            "run_id": run_id,
                            "type": classification,
                            "cause": root_cause,
                            "trace_path": str(trace_path),
                            "report_path": str(report_path),
                            "log_path": str(stdout_path) if stdout_path.exists() else ""
                        })

                # Consolidated Status
                runs_n = cell_stats["runs_total"]
                passed = cell_stats["runs_passed"]
                sem_fail = cell_stats["runs_semantic_fail"]
                infra_fail = cell_stats["runs_infra_fail"]
                
                status_label = "MISSING"
                if runs_n > 0:
                    if passed == runs_n:
                        status_label = "PASS"
                    elif sem_fail > 0:
                         # Semantic failure takes precedence over flakiness for now?
                         # User said: FAIL if semantic fail > 0.
                         status_label = "FAIL"
                         if passed > 0:
                             status_label = "FLAKY (FAILS)"
                    elif infra_fail > 0 and passed == 0:
                        status_label = "INFRA_FAIL"
                    elif infra_fail > 0 and passed > 0:
                        status_label = "FLAKY (INFRA)"
                
                # Calculate latency stats for the cell
                latency_str = ""
                if cell_stats["latencies"]:
                    lalk = sorted(cell_stats["latencies"])
                    min_l = lalk[0]
                    max_l = lalk[-1]
                    med_l = lalk[len(lalk)//2]
                    latency_str = f"Lat: {min_l:.2f}/{med_l:.2f}/{max_l:.2f} ms"

                notes_list = []
                if runs_n > 1:
                    notes_list.append("Reliability test")
                if latency_str:
                    notes_list.append(latency_str)
                if status_label == "FAIL" and backend == "rclpy" and probe in ["L05", "S11"]:
                    notes_list.append("(Strict Fidelity)")

                notes = " ".join(notes_list)

                cells.append({
                    "distro": distro,
                    "backend": backend,
                    "probe": probe,
                    "status": status_label,
                    "pass_rate": f"{passed}/{runs_n}",
                    "stats": cell_stats,
                    "notes": notes
                })

    # Sort cells
    cells.sort(key=lambda x: (x['distro'], x['backend'], x['probe']))
    
    # Generate Output
    write_aggregate_json(cells, evidence_dir / "aggregate.json")
    write_aggregate_md(cells, evidence_dir / "aggregate.md")
    write_reliability_report(cells, evidence_dir / "reliability_report.md")

    # Console Summary
    total_cells = len(cells)
    cells_passed = sum(1 for c in cells if c['status'] == 'PASS')
    cells_failed = sum(1 for c in cells if "FAIL" in c['status'] or "FLAKY" in c['status'])
    
    print(f"Aggregation complete. Processed {total_cells} cells.")
    print(f"Passed: {cells_passed}, Failed/Flaky: {cells_failed}")

def write_aggregate_json(cells, path):
    summary = {
        "cells": cells
    }
    with open(path, 'w') as f:
        json.dump(summary, f, indent=2)

def write_aggregate_md(cells, path):
    total_cells = len(cells)
    cells_passed = sum(1 for c in cells if c['status'] == 'PASS')
    cells_failed = sum(1 for c in cells if "FAIL" in c['status'] or "FLAKY" in c['status'])
    
    with open(path, 'w') as f:
        f.write(f"# Probe Matrix Reliability Results\n\n")
        f.write(f"**Total Cells**: {total_cells} | **Passed**: {cells_passed} | **Failed**: {cells_failed}\n\n")
        f.write("| Distro | Backend | Probe | Status | Pass Rate | Notes |\n")
        f.write("|---|---|---|---|---|---|\n")
        for c in cells:
            icon = "✅" if c['status'] == "PASS" else "❌" if "FAIL" in c['status'] else "⚠️"
            f.write(f"| {c['distro']} | {c['backend']} | {c['probe']} | {icon} {c['status']} | {c['pass_rate']} | {c['notes']} |\n")

def write_reliability_report(cells, path):
    with open(path, 'w') as f:
        f.write("# Reliability Diagnosis Report\n\n")
        
        # Table of Stability
        f.write("## Stability Summary\n\n")
        f.write("| Cell | Status | Pass Rate | Infra Fails | Semantic Fails |\n")
        f.write("|---|---|---|---|---|\n")
        for c in cells:
            name = f"{c['distro']}/{c['backend']}/{c['probe']}"
            s = c['stats']
            f.write(f"| {name} | {c['status']} | {c['pass_rate']} | {s['runs_infra_fail']} | {s['runs_semantic_fail']} |\n")
        
        f.write("\n## Failure Analysis\n\n")
        
        for c in cells:
            if c['stats']['failures']:
                name = f"{c['distro']}/{c['backend']}/{c['probe']}"
                f.write(f"### {name}\n")
                for fail in c['stats']['failures']:
                    f.write(f"- **{fail['run_id']}** ({fail['type']}): {fail['cause']}\n")
                    f.write(f"  - Trace: `{fail['trace_path']}`\n")
                    f.write(f"  - Report: `{fail['report_path']}`\n")
                    # Relative path for cleaner report if possible
                    log_p = fail['log_path']
                    if log_p:
                       f.write(f"  - Log: `{log_p}`\n")
                f.write("\n")

if __name__ == "__main__":
    main()
