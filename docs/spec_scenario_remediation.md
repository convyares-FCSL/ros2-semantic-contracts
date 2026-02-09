# Parameter Semantic Truth Discovery Plan (v5)

**Updated:** 2026-02-09  
**Status:** In progress (Phases 0-3 implemented; Phases 4-5 pending)  
**Scope:** Parameter scenarios `P01-P20`

## 1) What was completed

### Phase 0 (Evaluator correctness)
- Fixed `custom.must_observe` matching in `harness/core/src/eval.rs`:
  - Old behavior: independent `any(...)` matching (false-pass risk for duplicate patterns).
  - New behavior: ordered, unique-consumption matching.
- Added evaluator unit tests:
  - Duplicate pattern requires distinct events.
  - Out-of-order matching fails.
  - In-order matching passes.

### Phase 1 (Observation infrastructure)
- Added `parameter_event` to trace vocabulary:
  - `harness/schemas/trace_event.schema.json`
  - `harness/core/src/io.rs`
- Added new parameter operations in both backends:
  - `observe_param_events`
  - `register_param_callback`
  - `register_pre_set_callback`
  - `set_params_non_atomic`
  - `list_params`
- Added corresponding scenario op enums in `harness/schemas/scenario.schema.json`.
- Registered handlers and capabilities in both backends.

### Phase 2 (New discovery scenarios)
- Added `P17-P20` to `harness/scenarios/scenarios_P.json`.
- Baseline and comparison executed on both backends.

### Phase 3 (Proxy upgrades)
- Upgraded `P10`, `P11`, `P13`, `P14` to direct observation paths:
  - `P10`: explicit `observe_param_events` silence check after rejection.
  - `P11`: direct `parameter_event` payload assertions for value/type changes.
  - `P13`: direct `list_params` prefix/depth stability assertions.
  - `P14`: direct observation records no declare/undeclare event in current Jazzy setup, while state removal remains observable.

## 2) Measured findings (Jazzy, `backend_prod` vs `backend_py`)

### P17 (non-atomic batch + callback rejection)
- Observed in both backends:
  - `set_params_non_atomic` returns mixed results.
  - `p17_param_a` applied (`a1`), `p17_param_b` rejected (`b0` remains).
- Result: **match**.

### P18 (atomic batch + callback rejection)
- Observed in both backends:
  - Atomic batch rejected (`successful: false`).
  - Neither value changed (`a0`, `b0` preserved).
- Result: **match**.

### P19 (pre-set callback removal)
- Observed in both backends:
  - `p19_param_b` removed by pre-set callback.
  - Batch succeeds; `p19_param_a` updates, `p19_param_b` remains unchanged.
- Result: **match**.

### P20 (parameter event cardinality for atomic batch)
- Observed in both backends:
  - One `parameter_event` emitted for 3-parameter atomic update.
  - Event payload includes all 3 entries in `changed_parameters`.
- Result: **match**.

## 3) Determinism summary

- `P17-P20` validated with repeated runs on both backends.
- 5 successful runs produced identical report hashes on both backends for the focused `P17-P20` bundle.
- One `backend_prod` attempt hit a peer sentinel race (`INFRA_FAIL`), then reruns succeeded and remained hash-stable.
- Full `P01-P20` suite passes on both backends.

## 4) Remaining work

### Phase 4 (spec/scenario alignment)
- Update spec prose where claims implied by proxy tests are now replaced by direct evidence.
- Reconcile wording around deletion-event semantics with measured Jazzy behavior.

### Phase 5 (confidence publication)
- Update scenario matrix with:
  - `P17-P20`
  - direct vs proxy confidence notes for all parameter rows.
