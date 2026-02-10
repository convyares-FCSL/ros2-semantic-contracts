# Spec-Scenario Realignment

## Phase 0: Evidence Integrity ✅
- [x] Audit all P-series `must_observe` patterns for missing value assertions — evaluator is sound
- [x] No false-pass risk found — consume+order semantics prevent duplicate matching

## Phase 1: Spec Completeness ✅
- [x] Add SPEC_PC09 — Descriptor state after set (UNVALIDATED)
- [x] Add SPEC_PC10 — Explicit descriptor type enforcement (UNVALIDATED)
- [x] Add SPEC_PC11 — Declaration without default value (UNVALIDATED)
- [x] Add SPEC_PC12 — GetParameterTypes response contract (UNVALIDATED)
- [x] Fix SPEC_P08/PC08 — Deletion events MUST→SHOULD

## Phase 2: Derive Scenarios + Implement
- [x] P21 scenario JSON — descriptor type tracking (uses existing ops)
- [x] P22 scenario JSON — descriptor.type enforcement (needs declare_param enhancement)
- [x] P23 scenario JSON — declare without default (needs declare_param enhancement)
- [x] P24 scenario JSON — get_parameter_types response (needs new op)
- [x] Fix P14 notes to reference corrected SPEC_P08
- [ ] P22: enhance `declare_param` in both backends to accept `descriptor.type`
- [ ] P23: enhance `declare_param` in both backends for no-default mode
- [ ] P24: new `get_parameter_types` op in both backends + dispatch
- [ ] P25: new `concurrent_read_during_batch` op in both backends + scenario JSON
- [ ] `colcon build` both backends
- [ ] Each new scenario runs without crash or trace schema violation

## Phase 3: Measure
- [ ] Run P21-P25 on backend_prod (rclcpp) — verify or correct spec hypotheses
- [ ] Run P21-P25 on backend_py (rclpy) — record results
- [ ] Regression: P01-P20 still pass for both backends

## Phase 4: Document
- [ ] Update scenario_execution_matrix.md with observed results
- [ ] Update missing_semantics.md
- [ ] File divergence records for confirmed divergences
