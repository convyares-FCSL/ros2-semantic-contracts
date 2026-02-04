#!/usr/bin/env python3
"""
Aggregates reliability test results into summary JSON and Markdown.

Reads from evidence/reliability/<distro>/<backend>/run_*/
Writes to evidence/aggregate.json and evidence/aggregate.md
"""

import json
import os
from pathlib import Path
from typing import Dict, List, Any

# Configuration
MIN_TRACE_BYTES = 10


def main():
    evidence_dir = Path("evidence")
    reliability_dir = evidence_dir / "reliability"

    if not reliability_dir.exists():
        print("No reliability evidence found. Run scripts/run_reliability.sh first.")
        return

    cells = []

    # Discover all cells: evidence/reliability/<distro>/<backend>/
    for distro_dir in sorted(reliability_dir.iterdir()):
        if not distro_dir.is_dir():
            continue
        distro = distro_dir.name

        for backend_dir in sorted(distro_dir.iterdir()):
            if not backend_dir.is_dir():
                continue
            backend = backend_dir.name

            cell_stats = process_cell(distro, backend, backend_dir)
            cells.append(cell_stats)

    # Write outputs
    write_aggregate_json(cells, evidence_dir / "aggregate.json")
    write_aggregate_md(cells, evidence_dir / "aggregate.md")

    # Console summary
    total_runs = sum(c["runs_total"] for c in cells)
    total_pass = sum(c["runs_pass"] for c in cells)
    total_fail = sum(c["runs_semantic_fail"] for c in cells)
    total_infra = sum(c["runs_infra_fail"] for c in cells)

    print(f"Aggregation complete: {len(cells)} cell(s), {total_runs} run(s)")
    print(f"  PASS: {total_pass}, SEMANTIC_FAIL: {total_fail}, INFRA_FAIL: {total_infra}")


def process_cell(distro: str, backend: str, cell_dir: Path) -> Dict[str, Any]:
    """Process all runs in a cell directory."""
    stats = {
        "distro": distro,
        "backend": backend,
        "runs_total": 0,
        "runs_pass": 0,
        "runs_semantic_fail": 0,
        "runs_infra_fail": 0,
        "scenarios": {},  # Per-scenario stats
        "failures": [],
    }

    run_dirs = sorted([d for d in cell_dir.iterdir() if d.is_dir() and d.name.startswith("run_")])

    for run_dir in run_dirs:
        stats["runs_total"] += 1
        run_result = process_run(run_dir, stats)

    # Determine overall cell status
    if stats["runs_total"] == 0:
        stats["status"] = "MISSING"
    elif stats["runs_infra_fail"] == stats["runs_total"]:
        stats["status"] = "INFRA_FAIL"
    elif stats["runs_pass"] == stats["runs_total"]:
        stats["status"] = "PASS"
    elif stats["runs_semantic_fail"] > 0:
        if stats["runs_pass"] > 0:
            stats["status"] = "FLAKY"
        else:
            stats["status"] = "FAIL"
    else:
        stats["status"] = "FLAKY"

    stats["pass_rate"] = f"{stats['runs_pass']}/{stats['runs_total']}"

    return stats


def process_run(run_dir: Path, stats: Dict[str, Any]) -> str:
    """Process a single run, update stats, return classification."""
    run_id = run_dir.name
    trace_path = run_dir / "trace.jsonl"
    report_path = run_dir / "report.json"
    result_path = run_dir / "result.txt"

    # Evidence validity gate
    gate_failure = None

    if not trace_path.exists():
        gate_failure = "missing trace.jsonl"
    elif trace_path.stat().st_size < MIN_TRACE_BYTES:
        gate_failure = "empty trace.jsonl"

    if gate_failure is None and not report_path.exists():
        gate_failure = "missing report.json"

    report_data = None
    if gate_failure is None:
        try:
            with open(report_path) as f:
                report_data = json.load(f)
        except json.JSONDecodeError as e:
            gate_failure = f"invalid JSON in report.json: {e}"

    if gate_failure is None and report_data:
        scenarios = report_data.get("scenarios", {})
        if not scenarios:
            gate_failure = "no scenarios in report"

    # Classification
    if gate_failure:
        stats["runs_infra_fail"] += 1
        stats["failures"].append({
            "run_id": run_id,
            "type": "INFRA_FAIL",
            "cause": gate_failure,
            "trace_path": str(trace_path),
            "report_path": str(report_path),
        })
        return "INFRA_FAIL"

    # Check report outcome
    all_pass = report_data.get("pass", False)

    if all_pass:
        stats["runs_pass"] += 1
        # Update per-scenario stats
        for scen_id, scen_data in report_data.get("scenarios", {}).items():
            if scen_id not in stats["scenarios"]:
                stats["scenarios"][scen_id] = {"pass": 0, "fail": 0, "skip": 0}
            outcome = scen_data.get("outcome", "UNKNOWN")
            if outcome == "PASS":
                stats["scenarios"][scen_id]["pass"] += 1
            elif outcome == "FAIL":
                stats["scenarios"][scen_id]["fail"] += 1
            elif outcome == "SKIP":
                stats["scenarios"][scen_id]["skip"] += 1
        return "PASS"
    else:
        stats["runs_semantic_fail"] += 1
        # Find which scenarios failed
        failed_scenarios = []
        for scen_id, scen_data in report_data.get("scenarios", {}).items():
            if scen_id not in stats["scenarios"]:
                stats["scenarios"][scen_id] = {"pass": 0, "fail": 0, "skip": 0}
            outcome = scen_data.get("outcome", "UNKNOWN")
            if outcome == "PASS":
                stats["scenarios"][scen_id]["pass"] += 1
            elif outcome == "FAIL":
                stats["scenarios"][scen_id]["fail"] += 1
                failed_scenarios.append(scen_id)
            elif outcome == "SKIP":
                stats["scenarios"][scen_id]["skip"] += 1

        stats["failures"].append({
            "run_id": run_id,
            "type": "SEMANTIC_FAIL",
            "cause": f"scenarios failed: {', '.join(failed_scenarios)}",
            "trace_path": str(trace_path),
            "report_path": str(report_path),
        })
        return "SEMANTIC_FAIL"


def write_aggregate_json(cells: List[Dict], path: Path):
    """Write aggregate.json with full cell data."""
    summary = {
        "version": "0.1",
        "cells": cells,
        "totals": {
            "cells": len(cells),
            "runs": sum(c["runs_total"] for c in cells),
            "pass": sum(c["runs_pass"] for c in cells),
            "semantic_fail": sum(c["runs_semantic_fail"] for c in cells),
            "infra_fail": sum(c["runs_infra_fail"] for c in cells),
        }
    }
    with open(path, "w") as f:
        json.dump(summary, f, indent=2)


def write_aggregate_md(cells: List[Dict], path: Path):
    """Write aggregate.md with human-readable summary."""
    total_runs = sum(c["runs_total"] for c in cells)
    total_pass = sum(c["runs_pass"] for c in cells)
    total_fail = sum(c["runs_semantic_fail"] for c in cells)
    total_infra = sum(c["runs_infra_fail"] for c in cells)

    with open(path, "w") as f:
        f.write("# Reliability Test Results\n\n")
        f.write(f"**Total Runs**: {total_runs} | ")
        f.write(f"**Pass**: {total_pass} | ")
        f.write(f"**Semantic Fail**: {total_fail} | ")
        f.write(f"**Infra Fail**: {total_infra}\n\n")

        f.write("## Summary Table\n\n")
        f.write("| Distro | Backend | Status | Pass Rate | Infra Fail | Semantic Fail |\n")
        f.write("|--------|---------|--------|-----------|------------|---------------|\n")

        for c in cells:
            icon = {"PASS": "✓", "FAIL": "✗", "FLAKY": "~", "INFRA_FAIL": "!"}.get(c["status"], "?")
            f.write(f"| {c['distro']} | {c['backend']} | {icon} {c['status']} | ")
            f.write(f"{c['pass_rate']} | {c['runs_infra_fail']} | {c['runs_semantic_fail']} |\n")

        # Per-scenario breakdown if available
        f.write("\n## Per-Scenario Breakdown\n\n")
        for c in cells:
            if c["scenarios"]:
                f.write(f"### {c['distro']}/{c['backend']}\n\n")
                f.write("| Scenario | Pass | Fail | Skip |\n")
                f.write("|----------|------|------|------|\n")
                for scen_id, scen_stats in sorted(c["scenarios"].items()):
                    f.write(f"| {scen_id} | {scen_stats['pass']} | {scen_stats['fail']} | {scen_stats['skip']} |\n")
                f.write("\n")

        # Failure details
        any_failures = any(c["failures"] for c in cells)
        if any_failures:
            f.write("## Failure Details\n\n")
            for c in cells:
                if c["failures"]:
                    f.write(f"### {c['distro']}/{c['backend']}\n\n")
                    for fail in c["failures"]:
                        f.write(f"- **{fail['run_id']}** ({fail['type']}): {fail['cause']}\n")
                        f.write(f"  - Trace: `{fail['trace_path']}`\n")
                        f.write(f"  - Report: `{fail['report_path']}`\n")
                    f.write("\n")


if __name__ == "__main__":
    main()
