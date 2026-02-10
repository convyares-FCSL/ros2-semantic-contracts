# Spec-to-Scenario Status Matrix

Last updated: 2026-02-10

Scope:
- Closure-focused matrix for parameter specs tied to active divergences and the set-parameter closure axis.
- Full scenario inventory remains in `docs/provenance/scenario/scenario_execution_matrix.md`.

Legend:
- `✅` = pass
- `❌` = fail
- `⚠️` = mixed/divergent across cells

## A) Active Divergence Specs

| Spec | Scenario | Jazzy cpp | Jazzy py | Humble cpp | Humble py | Status |
|---|---|---|---|---|---|---|
| `SPEC_PC12` | `P24_get_parameter_types_response_length` | ❌ | ❌ | ❌ | ❌ | ❌ Contract violation on both libraries |
| `SPEC_PC11` | `P31_no_default_with_startup_override` | ✅ | ❌ | ✅ | ✅ | ⚠️ Jazzy `rclpy` regression |
| `SPEC_P05` | `P32_startup_declare_event_batching` | ❌ | ❌ | ❌ | ❌ | ❌ Shared startup batching gap |
| `SPEC_PC14` | `P25_concurrent_visibility_probe` | ✅ | ❌ | ✅ | ❌ | ⚠️ Non-atomic visibility asymmetry (`prod` vs `py`) |
| `SPEC_PC14` | `P35_concurrent_visibility_atomic_control` | ✅ | ✅ | ✅ | ✅ | ✅ Atomic control passes everywhere |

## B) Set-Parameter Semantics Closure (`SPEC_PC16`)

| Spec | Scenario | Jazzy cpp | Jazzy py | Humble cpp | Humble py | Status |
|---|---|---|---|---|---|---|
| `SPEC_PC16` | `P27_standard_batch_partial_application_check` | ✅ | ✅ | ✅ | ✅ | ✅ |
| `SPEC_PC16` | `P28_atomic_batch_transaction_check` | ✅ | ✅ | ✅ | ✅ | ✅ |
| `SPEC_PC16` | `P29_callback_storage_old_value_visibility` | ✅ | ✅ | ✅ | ✅ | ✅ |
| `SPEC_PC16` | `P30_external_visibility_after_success_only` | ✅ | ✅ | ✅ | ✅ | ✅ |
| `SPEC_PC16` | `P33_standard_batch_partial_application_type_failure` | ✅ | ✅ | ✅ | ✅ | ✅ |
| `SPEC_PC16` | `P34_atomic_batch_transaction_integrity_type_failure` | ✅ | ✅ | ✅ | ✅ | ✅ |

Evidence basis:
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/report.json`
- `docs/provenance/scenario/scenario_execution_matrix.md`
