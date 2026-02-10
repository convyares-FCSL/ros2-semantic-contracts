# docs/ — Project Documentation

This directory contains the normative specifications, harness documentation, provenance records, and foundational design documents for **ros2-semantic-contracts**.

Start here to orient yourself. For project-level setup and execution, see the repository root README.

---

## Directory Map

### `spec/`
Normative semantic contracts — the source of truth for required ROS 2 behaviour.

| Path | Contents |
|------|----------|
| `spec/core/` | Core engine-enforceable semantic invariants |
| `spec/global/` | ROS-facing observable semantics |
| `spec/system/` | Interoperability and system-level contracts |
| `spec/spec_auth_matrix.md` | Authority ladder — what sources back each spec |
| `spec/spec_scenario_matrix.md` | Mapping from specs to executable scenarios |
| `spec/README.md` | Spec conventions and reading guide |

### `provenance/`
Design rationale, traceability, and evidence artifacts that link specs to scenarios and results.

| Path | Contents |
|------|----------|
| `provenance/scenario/` | Scenario design documents and provenance records |
| `provenance/oracle/` | Oracle boundary rules and evaluation semantics |
| `provenance/divergence_records/` | Documented mismatches between spec and observed behaviour |
| `provenance/reports/` | Audit reports and summary evidence |
| `provenance/missing_semantics.md` | Known gaps — behaviour not yet covered by specs |
| `provenance/README.md` | Provenance conventions |

### `harness/`
Documentation for the oracle harness — execution rules, reliability constraints, and evidence gates.

| Path | Contents |
|------|----------|
| `harness/RELIABILITY_RULES.md` | Evidence gates, exit-code semantics, no-cheating constraints |

### `_work/`
Scratch/working area. Not normative.

---

## Foundational Documents

These three documents define the project's authority model, process rules, and design constraints. They are peers, not a hierarchy.

| Document | Answers | Summary |
|----------|---------|---------|
| [intent.md](intent.md) | *Why does this exist?* | Scope, goals, success criteria, and what this project refuses to be |
| [philosophy.md](philosophy.md) | *What rules govern semantic truth?* | Authority sources, the role of Nav2, how non-determinism and failure are handled |
| [methodology.md](methodology.md) | *How are contracts produced and evolved?* | Authoring rules, provenance tagging, hypothesis handling, versioning, divergence management |

---

## Key Concepts (Quick Reference)

**Specs** define required behaviour using normative language (MUST / SHALL / MUST NOT). They live in `spec/`.

**Scenarios** are executable test bundles derived from specs. Design docs live in `provenance/scenario/`; runtime bundles live in `harness/scenarios/` (at the repo root, not here).

**The Oracle** evaluates a trace against a scenario bundle and produces a report. It does not define truth — it detects divergence from the specs.

**Evidence** is the output chain: `trace.jsonl` → `report.json` → human-facing artifacts in `evidence/` (repo root). During Stage-2 development, execution artifacts live in `/tmp` and CI; they are not committed.

**Backends** implement the system under test. Each backend declares its capabilities at runtime. The harness scores based on what was declared:
- **SKIP** — backend does not support this capability.
- **FAIL** — backend declared support but violated the contract.

---

## Execution Quick Reference

Backend CLI contract:
```
backend_<name> <bundle_path> <trace_path>
```

Oracle CLI contract:
```
oracle_core <bundle_path> <trace_path> <report_path> --backend <backend_bin>
```

Exit codes: `0` = all pass, `1` = scenario(s) FAIL, `2` = bundle error, `4` = system error.

---

## Current Baseline

**Profile:** ROS 2 Jazzy + rclcpp
**Stage:** Stage-2 (ros_local validation — specs are hypotheses until baseline-validated)
**Smoke scenario:** H00 (in `scenarios_H.json`)
