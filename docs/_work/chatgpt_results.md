# ChatGPT Results — Phase 2/3/4

## 1) P21-P25 Results

| Scenario | backend_prod (rclcpp) | backend_py (rclpy) | Notes |
| --- | --- | --- | --- |
| `P21_descriptor_tracks_value_type` | PASS | PASS | Descriptor `type` tracked dynamic value-type transitions in both backends. |
| `P22_explicit_descriptor_type_enforcement` | PASS | PASS | Explicit descriptor type was preserved on declare in both backends. |
| `P23_declare_without_default` | PASS | PASS | Explicitly typed declaration without default succeeded; later read was `NOT_SET`. |
| `P24_get_parameter_types_response_length` | PASS | PASS | Mixed declared/undeclared request failed with `NotDeclared` in both backends. |
| `P25_concurrent_visibility_probe` | PASS | FAIL | Updated single-call probe: `prod` observed mixed snapshots (`partial_state_observed=true`), `py` observed none (`false`) and fails against baseline expectation. |
| `P35_concurrent_visibility_atomic_control` | PASS | PASS | Atomic control probe observed no mixed snapshots in either backend (`partial_state_observed=false`). |

## 2) Divergence Records Created

No new divergence records were created in this phase.

`docs/provenance/divergence_records/` remains unchanged (`README.md` only).

## 3) Spec Hypotheses Corrected

Baseline measurement required these corrections:

- `SPEC_PC12` in `docs/spec/core/parameter_core.md`
  - Updated from 1:1 mapping hypothesis to observed Jazzy behavior: mixed declared/undeclared `get_parameter_types` fails with `NotDeclared`.
- `SPEC_P03` in `docs/spec/global/parameters.md`
  - Split behavior by API: `get_parameters` unknown -> `NOT_SET`; `get_parameter_types` mixed undeclared -> `NotDeclared` failure.
- `P23` expectation in `harness/scenarios/scenarios_P.json`
  - Corrected expected `get_param.type` from `not_set` to `NOT_SET` to match trace contract.
- `P24` expectation in `harness/scenarios/scenarios_P.json`
  - Corrected expected outcome to `successful=false`, `reason=NotDeclared`.
- `P25` probe notes/expectation in `harness/scenarios/scenarios_P.json`
  - Updated to measured baseline for this bulk-read probe (`partial_state_observed=false`).

## 4) Scenarios That Could Not Run

None in the final measured state.

Interim blockers encountered and resolved during implementation:

- Unhandled uninitialized-parameter reads (`P23`) in both backends caused backend termination.
- Unhandled `get_parameter_types` undeclared-name failure (`P24`) in backend_prod caused backend termination.
- C++ parameter scenario teardown hang during event-observation sequence was resolved by using `spin_some` polling for parameter events and only enabling background spinner for action scenarios.

## 5) Regression Results for P01-P20

- backend_prod: P01-P20 all PASS (`0` failures).
- backend_py: P01-P20 all PASS (`0` failures).

## Determinism Check

Three repeated runs per backend produced identical sorted-report hashes:

- backend_prod: `108c61cbe916e1a16bf985b5025b27fb` (all 3 runs)
- backend_py: `108c61cbe916e1a16bf985b5025b27fb` (all 3 runs)

## Trace Comparison Note (P21-P25)

Field-by-field `op_end` comparison between backends matched except for one instrumentation field:

- `P25` `concurrent_read_during_batch.detail.snapshot_count`
  - backend_prod: `2`
  - backend_py: `1`

Outcome fields remained aligned (`successful=true`, `partial_state_observed=false`).
