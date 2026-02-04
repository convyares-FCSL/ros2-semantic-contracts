# Project Context — ros2-semantic-contracts

## What this is
A contract-testing harness for ROS 2 semantics. We encode normative behavior as scenarios,
execute them against backends, and evaluate traces to produce reports.

Goal: find and document real divergence across {language} × {distro}, and lock in reproducible evidence.

## Key concepts
### Specs
Location: docs/spec/
Purpose: normative statements (“MUST/SHOULD/MUST NOT”), definitions, and contract intent.

### Scenarios
Location: docs/provenance/scenario/ (design/provenance)
Runtime bundles: harness/scenarios/ (actual JSON consumed by harness)
Purpose: executable examples derived from specs. A scenario defines:
- required capabilities
- ops (what backend must do)
- expects (what trace must show)

### Evidence
Location: evidence/ (human-facing artifacts: audit reports, PDFs, summaries)
Runtime artifacts:
- trace.jsonl (append-only JSON lines of events)
- report.json (oracle evaluation output)
Per-run trace/report paths are runtime-dependent (Docker mounts, /tmp, etc.).

## Harness
Location: harness/
- core/ : oracle evaluator (reads bundle + trace, writes report)
- scripts/run_harness.sh : orchestration entrypoint
- backends/ : backend implementations

### Backends (implemented)
- backend_stub : Rust reference stub for early validation
- backend_ros  : Rust harness backend (not rclrs; used for baseline smoke tests)
- backend_prod : production C++ backend (rclcpp) — the long-term baseline

### Backends (planned, not yet implemented)
- backend_rclpy : Python probe backend (Track 2, gated on 8/10 Track 1 stable)
- Future candidates: rclrs, rclgo, etc.

## Execution contracts
Backend CLI: `backend_<name> <bundle_path> <trace_path>`
Core CLI: `oracle_core <bundle_path> <trace_path> <report_path> --backend <backend_bin>`
Exit codes: 0 = all pass, 1 = scenario(s) FAIL, 2 = bundle error, 4 = system error.

## Validation notes
- H00 (in scenarios_H.json) is the baseline smoke scenario.
- Docker validation uses 5 iterations to confirm determinism.
- Evidence validity gates reject missing/empty/invalid artifacts.
- See CLAUDE.md for enforced invariants.
- See docs/harness/RELIABILITY_RULES.md for detailed evidence gates, exit-code semantics, and no-cheating constraints.
- See docs/provenance/oracle/boundaries.md for SUT/Peer separation rules (Actions enforced).
