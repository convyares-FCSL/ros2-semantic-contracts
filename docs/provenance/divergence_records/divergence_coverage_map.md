# Divergence Coverage Map

> Canonical coverage dashboard for parameter divergence closure.

Last updated: 2026-02-10  
Columns:
- `Jazzy cpp` = Jazzy `prod`
- `Jazzy py` = Jazzy `py`
- `Humble cpp` = Humble `prod`
- `Humble py` = Humble `py`
Legend:
- `✅` = pass / confirmed
- `❌` = fail
- `⚠️` = open divergence / caution

## 1) GitHub Issues (Clear Status)

| Issue | Scenario | Jazzy cpp | Jazzy py | Humble cpp | Humble py | Status | What it means |
|---|---|---|---|---|---|---|---|
| `ros2/rclcpp#2512` | `P24` | ❌ | ❌ | ❌ | ❌ | ⚠️ Open, reproduced | Both libraries violate 1:1 `get_parameter_types` contract on mixed declared+undeclared requests. |
| `ros2/rclpy#1460` | `P31` | ✅ | ❌ | ✅ | ✅ | ⚠️ Open, reproduced | Exact CLI path regression exists on Jazzy `rclpy` only. |
| `ros2/rclcpp#1970` | `P32` | ❌ | ❌ | ❌ | ❌ | ⚠️ Open, reproduced | Startup `declare_parameter` calls are not batched into a single parameter event. |
| `ros2/rclcpp#728` | `P36`,`P37` | ✅ | ✅ | ✅ | ✅ | ✅ Confirmed closed behavior | Range enforcement is present in measured matrix. |

## 2) Known Parameter Divergences (4)

These are the four parameter-level divergences currently tracked in this repo.

| Divergence | Scenario | Jazzy cpp | Jazzy py | Humble cpp | Humble py | State |
|---|---|---|---|---|---|---|
| Service contract mismatch (`get_parameter_types` mapping) | `P24` | ❌ | ❌ | ❌ | ❌ | ⚠️ Open divergence (both libs) |
| Typed no-default + CLI override regression | `P31` | ✅ | ❌ | ✅ | ✅ | ⚠️ Open divergence (Jazzy `py`) |
| Startup declaration event batching gap | `P32` | ❌ | ❌ | ❌ | ❌ | ⚠️ Open divergence (both libs) |
| Non-atomic concurrent visibility asymmetry | `P25` | ✅ | ❌ | ✅ | ❌ | ⚠️ Open divergence (`prod` vs `py`) |

## 3) Set-Parameter Closure (Single Spec, Two Scenarios)

Target axis to close out:
- Spec: `SPEC_PC14` (Concurrent Read Visibility Profiles)
- Scenario pair: one non-atomic probe (`P25`) + one atomic control (`P35`)

| Spec | Scenario | Jazzy cpp | Jazzy py | Humble cpp | Humble py | Result |
|---|---|---|---|---|---|---|
| `SPEC_PC14` | `P25` non-atomic concurrent visibility probe | ✅ | ❌ | ✅ | ❌ | ⚠️ Divergence is real and Python-specific in measured matrix. |
| `SPEC_PC14` | `P35` atomic control probe | ✅ | ✅ | ✅ | ✅ | ✅ Control passes everywhere (expected baseline behavior). |

Decision:
- Set-parameter divergence is not generic API breakage.
- It is scoped to non-atomic concurrent visibility (`P25`), with atomic control (`P35`) passing across all four cells.

Boundary note:
- `SPEC_PC16` behavioral semantics (`P27`,`P28`,`P29`,`P30`,`P33`,`P34`) are all parity passes in the current matrix and are not the active divergence.

## 4) Evidence

Issue matrix evidence:
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/report.json`

Divergence records:
- `docs/provenance/divergence_records/DIV_001_crosslib_get_parameter_types_contract_violation.md`
- `docs/provenance/divergence_records/DIV_002_crosslib_startup_declare_event_batching_gap.md`
- `docs/provenance/divergence_records/DIV_003_rclpy_jazzy_cli_typed_no_default_regression.md`
- `docs/provenance/divergence_records/DIV_004_crosslib_non_atomic_visibility_asymmetry.md`

## 5) Future Known Issues / Watchlist (Restored)

These are tracked as forward-looking items so they do not disappear from planning context.

| Topic | Current state | Why it stays on watch |
|---|---|---|
| Upstream issue for `SPEC_PC14` / `P25` asymmetry | ⚠️ No linked GitHub issue yet | Divergence is reproduced in both distros (`prod` vs `py`) and should be mapped to an upstream thread or filed with a minimal repro. |
| `ros2/rclcpp#728` regression watch | ✅ Closed upstream; `P36`,`P37` passing | Keep as a sentinel to detect future regressions in descriptor range enforcement. |
