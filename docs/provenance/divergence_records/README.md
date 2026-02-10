# Divergence Records

This directory contains the **official record of semantic differences** between the Baseline Authority (Jazzy+rclcpp) and other implementations.

> **Purpose:** To allow engineers to make informed decisions about which client library or distribution fits their strict requirements.

---

## Divergence Classifications

When a backend fails a scenario that the Baseline passes, classify the divergence as:

### 1. Defect (`TYPE: DEFECT`)
The implementation is objectively broken (e.g., it crashes, deadlocks, or violates the core state machine).
* **Action:** File issue upstream. Track fix.

### 2. Missing Feature (`TYPE: GAP`)
The implementation has not yet implemented a spec feature (e.g., "Parameter events not yet supported").
* **Action:** Document limitation. Update `coverage_matrix` to SKIP.

### 3. Intentional Deviation (`TYPE: FORK`)
The implementation explicitly chooses a different behavior for architectural reasons (e.g., "Go client uses channels instead of callbacks").
* **Action:** Document the behavioral difference. The Spec may need a "Variant" clause if this behavior is valid.

### 4. Baseline Regression (`TYPE: REGRESSION`)
A newer version (e.g., Rolling) broke a behavior present in the Baseline (Jazzy).
* **Action:** Critical alert. This breaks ecosystem stability.

---

## Record Format

Records are immutable Markdown files named:
- `DIV_<ID>_<scope>_<short_desc>.md`

Recommended structure:
1. Metadata
2. Core finding (exact semantic mismatch)
3. Upstream evidence and history (issue links, status, timeline)
4. Matrix evidence (pinned run results + trace signatures)
5. Impact
6. Disposition

### Required Metadata Fields

- `Date`
- `Type` (`DEFECT` / `GAP` / `FORK` / `REGRESSION`)
- `Spec/Scenario`
- `Related upstream` links with current status

### Evidence Rules

- Record only issue-equivalent or otherwise decisive scenarios.
- Always include pinned matrix paths:
  - `evidence/reliability/jazzy/prod/run_01/report.json`
  - `evidence/reliability/jazzy/py/run_01/report.json`
  - `evidence/reliability/humble/prod/run_01/report.json`
  - `evidence/reliability/humble/py/run_01/report.json`
- Include at least one trace signature snippet path that directly shows the divergence.

### Clarity Rules

- Distinguish clearly:
  - contract violation (`DEFECT`) vs
  - missing feature (`GAP`) vs
  - distro/library break (`REGRESSION`)
- State whether the divergence is:
  - cross-library (both libs), or
  - library-specific, and on which distro.

---

## Active Records

| Record | Spec/Scenario | Upstream | Current status |
|---|---|---|---|
| `DIV_001_crosslib_get_parameter_types_contract_violation.md` | `SPEC_PC12` / `P24` | `ros2/rclcpp#2512` | Open upstream |
| `DIV_002_crosslib_startup_declare_event_batching_gap.md` | `SPEC_P05` / `P32` | `ros2/rclcpp#1970` | Open upstream |
| `DIV_003_rclpy_jazzy_cli_typed_no_default_regression.md` | `SPEC_PC11` / `P31` | `ros2/rclpy#1460` | Open upstream |
| `DIV_004_crosslib_non_atomic_visibility_asymmetry.md` | `SPEC_PC14` / `P25`,`P35` | No mapped issue yet | Open (watchlist, upstream mapping pending) |

## Coverage Dashboard

- `docs/provenance/divergence_records/divergence_coverage_map.md`
