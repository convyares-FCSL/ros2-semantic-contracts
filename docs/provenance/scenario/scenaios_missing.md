# Parameter Coverage Gaps and Missing Work

Last updated: 2026-02-10

Scope:
- Parameter-only gap register.
- Replaces prior ad-hoc AI notes with current, evidence-linked status.

## 1) Scenario inventory completeness

- Implemented parameter scenarios in bundle: `40` (`P01`-`P42`, excluding unused IDs).
- Missing scenario IDs for currently retained parameter specs: `None`.

Interpretation:
- The current gap is no longer "missing scenarios".
- The gap is now mostly point polarity completeness, divergence closure, and infra execution coverage.

## 2) Missing polarity or deferred point coverage

| Spec point | Current evidence | Gap type | Required follow-up |
|---|---|---|---|
| `SPEC_PC06.P2` | `P38` | Happy-only | Add explicit negative/control for undeclared-allowed mode boundaries. |
| `SPEC_PC09.P1` | `P21` | Happy-only | Add control/negative for descriptor-sync counterfactual. |
| `SPEC_PC10.P1` | `P22` | Happy-only | Add declare-time edge control to close polarity. |
| `SPEC_P02.P2` | `P38` | Happy-only | Add negative/control pair for undeclared-allowed profile boundaries. |
| `SPEC_P01.P1` | `P15` (proxy) | Deferred | Add direct graph/topology primitive for explicit interface conformance. |
| `SPEC_P03.P2` | `P24` (profile note) | Deferred | Keep as documentation/profile point under `SPEC_PC12` authority. |

## 3) Open divergence points (not missing scenarios)

| Spec point(s) | Scenario | Current state | Why still open |
|---|---|---|---|
| `SPEC_PC12.P1/.P2` | `P24` | Fail in all measured cells | Upstream/service contract divergence remains unresolved. |
| `SPEC_PC14.P1`, `SPEC_P09.P2` | `P25` (+ `P35` control) | `prod` pass / `py` fail | Non-atomic visibility asymmetry remains open. |
| `SPEC_PC11.P3` | `P31` | Jazzy `py` fail; others pass | Jazzy `rclpy` typed no-default startup override regression persists. |

## 4) Execution coverage blockers

| Area | Blocker | Impact |
|---|---|---|
| Humble `py` full-bundle reliability run | Backend/runtime error: `Node.list_parameters` missing | Prevents clean full-cell refresh for probe/closure rows in Humble `py` matrix. |

## 5) Parameter backlog priorities

Priority order for next parameter cycle:
1. Close happy-only point polarity gaps (`SPEC_PC06.P2`, `SPEC_PC09.P1`, `SPEC_PC10.P1`, `SPEC_P02.P2`).
2. Resolve infra blocker for Humble `py` full-bundle execution coverage.
3. Continue divergence closure work for `P24`, `P25`, `P31`.

Evidence basis:
- `docs/spec/spec_execution_matrix.md`
- `docs/spec/spec_scenario_matrix.md`
- `docs/provenance/scenario/scenario_execution_matrix.md`
- `docs/provenance/divergence_records/divergence_coverage_map.md`
