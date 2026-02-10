# Spec-Scenario Realignment Plan

> **Status**: Active  
> **Date**: 2026-02-09  
> **Authority**: Observable results from rclcpp+Jazzy baseline and the open codebase. Nothing else.

---

## Problem Statement

All 20 P-series parameter scenarios pass for both `backend_prod` (rclcpp) and `backend_py` (rclpy), but documented GitHub issues show known behavioral divergences. Three layers to address:

1. **Evidence integrity**: Verify the evaluator can reliably distinguish PASS from FAIL before adding scenarios.
2. **Spec completeness**: Specs cover the value layer but are silent on descriptor metadata — where known divergences live.
3. **Observation capability**: The harness is sequential-only. A concurrent reader op is needed to observe the non-atomic batch lock asymmetry.

---

## Phase 0: Evidence Integrity

Before adding new specs or scenarios, verify the harness can be trusted.

### Evaluator Audit

The `must_observe` matcher in `eval.rs` does consume matches and enforce ordering. But `matches_pattern` is a recursive subset matcher — loose patterns risk false passes. Actions:

1. **Audit**: Check every P-series pattern for missing value assertions
2. **Negative test**: Construct a pattern set that SHOULD miss, verify evaluator reports FAIL
3. **Fix if needed**: Only add evaluator features if the audit reveals actual false-pass risk

### Concurrent Reader Op

Implement `concurrent_read_during_batch`:
1. Reader thread polls `get_param` in a tight loop on target params
2. Main thread fires `set_params_non_atomic` batch
3. Op reports all intermediate states observed → `partial_state_observed: true/false`

Both backends use their native threading (C++ `std::thread`, Python `threading`). Minimal implementation — one new op, nothing else.

---

## Phase 1: Complete P-Series Specs

Add missing clauses to `parameter_core.md`. All new clauses are ⚠️ UNVALIDATED hypotheses — each must be verified against the rclcpp baseline before promotion.

### New Clauses (all UNVALIDATED)

**SPEC_PC09 — Descriptor State After Set**
Hypothesis: after a successful set on a dynamically typed parameter, `describe_parameters` reports a descriptor whose `type` field matches the current value's type.
To verify: declare dynamic param as string, set to integer, call `describe_parameters`, check `type` field.

**SPEC_PC10 — Explicit Descriptor Type Enforcement**
Hypothesis: when declared with explicit `ParameterDescriptor.type`, the declared type is enforced over value-based deduction.
To verify: declare with `ParameterDescriptor.type = BOOL`, pass string value, observe behavior.

**SPEC_PC11 — Declaration Without Default Value**
Hypothesis: declaring a static-typed parameter without a default does not unconditionally raise an exception when a value is available at runtime.
To verify: declare param without default, supply value via parameter override.

**SPEC_PC12 — GetParameterTypes Response Contract**
Hypothesis: `get_parameter_types` returns a response array with length equal to the request array, with `NOT_SET` for undeclared params.
To verify: request types for mix of declared + undeclared names, check response length.

### Spec Fixes

**SPEC_P08 / SPEC_PC08**: Change deletion event requirement from MUST → SHOULD. Jazzy rclcpp does not emit `ParameterEvent` on `undeclare_parameter`, top-level docs are silent, rclcpp behavior defines truth here.

---

## Phase 2: Derive Scenarios and Implement

Each spec clause produces one scenario. Scenario design and implementation happen together — a scenario isn't done until it can actually run.

| Spec | Scenario | What It Measures |
|---|---|---|
| SPEC_PC09 | **P21** | `describe_param` after `set_param` — does descriptor.type track? |
| SPEC_PC10 | **P22** | Declare with explicit `ParameterDescriptor.type` — enforced? |
| SPEC_PC11 | **P23** | Declare static-typed param without default — exception? |
| SPEC_PC12 | **P24** | `get_parameter_types` response length — matches request? |
| SPEC_PC05 | **P25** | `concurrent_read_during_batch` — partial state visible? |

### Implementation Work Per Scenario

**P21 — No new ops needed.** Uses existing `declare_param` (with `dynamic_typing: true`), `set_param`, `describe_param`. Just a new scenario entry in `scenarios_P.json`.

**P22 — Enhance `declare_param` to accept `descriptor.type`.**
- `backend_prod/src/ops/params.cpp`: Parse `descriptor.type` from payload, pass to `rcl_interfaces::msg::ParameterDescriptor`
- `backend_py/ops/params.py`: Same — pass `descriptor.type` to `ParameterDescriptor`
- New scenario entry in `scenarios_P.json`

**P23 — Enhance `declare_param` to support no-default declaration.**
- `backend_prod/src/ops/params.cpp`: Allow omitting `value` from payload, call `declare_parameter(name, descriptor)` without value
- `backend_py/ops/params.py`: Same path through rclpy API
- New scenario entry in `scenarios_P.json`

**P24 — New op: `get_parameter_types`.**
- `backend_prod/src/ops/params.cpp`: New `exec_get_parameter_types()` — calls `node->get_parameter_types(names)`, returns type array
- `backend_prod/src/backend.cpp`: Register op in dispatch
- `backend_py/ops/params.py`: New handler — calls `node.get_parameter_types(names)`
- `backend_py/backend.py`: Register in `OP_HANDLERS`
- `backend_capabilities.json`: Add capability if needed
- New scenario entry in `scenarios_P.json`

**P25 — New op: `concurrent_read_during_batch`.**
- `backend_prod/src/ops/params.cpp`: New `exec_concurrent_read_during_batch()` — spawns reader thread polling `get_parameter`, fires `set_parameters` (non-atomic) on main thread, collects intermediate states
- `backend_prod/src/backend.cpp`: Register op in dispatch
- `backend_py/ops/params.py`: Same with Python `threading.Thread`
- `backend_py/backend.py`: Register in `OP_HANDLERS`
- New scenario entry in `scenarios_P.json`

### Build Verification

After implementation, before measurement:
- `colcon build` succeeds for both backends
- Each new scenario runs (even if result is unknown) without crash or trace schema violation

---

## Phase 3: Measure

For each new scenario:

1. **Run against rclcpp baseline first**
   - PASS → promote spec clause from UNVALIDATED to VALIDATED
   - FAIL → spec hypothesis was wrong, correct to match observed rclcpp behavior
2. **Run against rclpy**
   - PASS → parity confirmed, document it
   - FAIL → file divergence record with exact commands to reproduce and raw trace diff
3. **Regression**: confirm P01-P20 still pass for both backends

Every divergence record includes: scenario ID, spec reference, exact reproduction commands, raw trace excerpts, and classification per `divergence_records/README.md`.

---

## Phase 4: Document

- Update `scenario_execution_matrix.md` with observed results
- Update `missing_semantics.md` — remove items validated, add cross-references
- File divergence records for confirmed failures

---

## Future: Other Series

Known divergences in C/E/G/S families follow the same pattern: create spec → derive scenario → measure → document. These are the roadmap after P-series methodology is proven:

- **C**: Clock jump recovery (rclpy#1490), negative duration (rclcpp#525)
- **E**: Executor ordering (Discourse), MT reentrancy (rclpy#1452), timeout=0 (rclpy#1056)
- **G**: Empty name handling (rclpy#1448)
- **S**: Use-after-destroy (rclpy#1491)

---

## Methodology

```
Hypothesis (UNVALIDATED clause) → Scenario → Run rclcpp baseline
  ↓ PASS                                           ↓ FAIL
Promote to VALIDATED                      Fix spec to match rclcpp
  ↓
Run rclpy
  ↓ PASS              ↓ FAIL
Parity confirmed    Divergence record (with reproduction trace)
```
