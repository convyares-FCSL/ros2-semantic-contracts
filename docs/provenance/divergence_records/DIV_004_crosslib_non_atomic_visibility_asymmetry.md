# DIV_004 — Cross-Library Non-Atomic `set_parameters` Concurrent Visibility Asymmetry

- Date: 2026-02-10
- Type: `FORK` (architectural divergence)
- Spec/Scenario: `SPEC_PC14`, `P25_concurrent_visibility_probe`, `P35_concurrent_visibility_atomic_control`
- Related upstream:
  - `ros2/rclpy#1056` (executor/spin behavior context): https://github.com/ros2/rclpy/issues/1056
  - `rclcpp` parameter internals (mutex-guarded implementation context): https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp

## 1) Core Finding

The divergence is scoped to concurrent read visibility during non-atomic batch updates.

Observed matrix:
- `P25` (non-atomic probe): `Jazzy prod=✅`, `Jazzy py=❌`, `Humble prod=✅`, `Humble py=❌`
- `P35` (atomic control): `Jazzy prod=✅`, `Jazzy py=✅`, `Humble prod=✅`, `Humble py=✅`

Direct trace signature (Jazzy run_01):
- `prod` `P25`: `partial_state_observed=true`, `mixed_snapshot_count=18`
- `py` `P25`: `partial_state_observed=false`, `mixed_snapshot_count=0`
- both `P35`: `partial_state_observed=false`

Interpretation:
- This is not generic set-API failure.
- The active divergence is specifically non-atomic visibility profile behavior under concurrent observation.

## 2) Evidence and Scope

Matrix authority:
- `docs/provenance/scenario/scenario_execution_matrix.md`

Pinned reliability evidence:
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/report.json`

Pinned trace signatures:
- `evidence/reliability/jazzy/prod/run_01/trace.jsonl`
- `evidence/reliability/jazzy/py/run_01/trace.jsonl`
- `evidence/reliability/humble/prod/run_01/trace.jsonl`
- `evidence/reliability/humble/py/run_01/trace.jsonl`

Inference note:
- It is plausible this asymmetry is driven by runtime/executor model differences (C++ mutex + multithread interleavings vs Python GIL/executor serialization effects), but this repo records the behavior as measured evidence first, not as proven root-cause.

## 3) Impact

- Tooling expecting incremental intermediate visibility may behave differently across libraries.
- Race probes calibrated on one library can miss behavior on the other.
- Safety-critical updates should use atomic API (`set_parameters_atomically`), where `P35` shows parity.

## 4) Disposition

- Repository: `TYPE=FORK` (measured architectural divergence on non-atomic concurrent visibility).
- Upstream: no direct mapped issue for `SPEC_PC14` yet; keep watchlist open in `docs/provenance/divergence_records/divergence_coverage_map.md`.
