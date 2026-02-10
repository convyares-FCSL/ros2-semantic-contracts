# Spec-to-Scenario Status Matrix

Last updated: 2026-02-10

Scope:
- Spec-driven status matrix (one row per spec) across Core, Global, and System specs.
- Answers: "what is the current status of all specs" using current scenario execution evidence.

Legend:
- Backend cells: `✅` pass, `❌` fail, `⚠️` mixed/divergent, `N/A` unexecuted.
- `Overall`: `✅` all executed paths pass, `❌` executed paths fail, `⚠️` mixed/divergent, `Deferred` no executable scenario evidence yet.

Snapshot:
- Total specs tracked: `82`
- Overall `✅`: `28` | `❌`: `1` | `⚠️`: `6` | `Deferred`: `47`

## Core Specs

| Spec | Auth class | Scenario anchor(s) | stub | ros | PROD | PY | rclrs | Overall | Notes |
|---|---|---|---|---|---|---|---|---|---|
| `SPEC_AC01` | Required | `A01`, `A03` | ✅ | ⚠️ | ✅ | N/A | N/A | ⚠️ | - |
| `SPEC_AC02` | TBD (not in spec_auth_matrix) | `A15` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_AC03` | Required | `A02` | ✅ | N/A | ✅ | N/A | N/A | ✅ | - |
| `SPEC_AC04` | Required | `A04`, `A05` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_AC05` | Required | `A06`, `A07` | N/A | N/A | ✅ | N/A | N/A | ✅ | - |
| `SPEC_AC06` | TBD (not in spec_auth_matrix) | `A08`, `A09` | N/A | N/A | ✅ | N/A | N/A | ✅ | - |
| `SPEC_AC07` | TBD (not in spec_auth_matrix) | `A10`, `A11` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_LC01` | Required | - | N/A | N/A | N/A | N/A | N/A | Deferred | No scenario anchor in spec heading. |
| `SPEC_LC02` | Required | `L01` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_LC03` | Required | `L03` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_LC04` | TBD (not in spec_auth_matrix) | `L04`, `L05` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_LC05` | Rot Candidate | `L06` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_PC01` | Required | `P01` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC02` | Required | `P01` | N/A | N/A | ✅ | ✅ | N/A | ✅ | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_PC03` | Required | `P02`, `P08`, `P26` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC04` | Required | `P03` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC05` | Required | `P04`, `P05`, `P18` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC06` | Required | `P06` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC07` | Bloat? (No upstream source) | - | N/A | N/A | N/A | N/A | N/A | Deferred | No scenario anchor in spec heading. |
| `SPEC_PC08` | Rot Candidate | `P14`, `P42` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC09` | Should Have | `P21` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC10` | Should Have | `P22` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC11` | Should Have | `P23`, `P31` | N/A | N/A | ✅ | ⚠️ | N/A | ⚠️ | - |
| `SPEC_PC12` | Required (Failing) | `P24` | N/A | N/A | ❌ | ❌ | N/A | ❌ | - |
| `SPEC_PC13` | Required | `P17` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC14` | Should Have | `P25`, `P35` | N/A | N/A | ✅ | ⚠️ | N/A | ⚠️ | - |
| `SPEC_PC15` | Required | `P26` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC16` | Should Have | `P27`, `P28`, `P29`, `P30`, `P33`, `P34`, `P41` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_PC17` | Required | `P36`, `P37` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |

## Global Specs

| Spec | Auth class | Scenario anchor(s) | stub | ros | PROD | PY | rclrs | Overall | Notes |
|---|---|---|---|---|---|---|---|---|---|
| `SPEC_A01` | TBD (not in spec_auth_matrix) | `A16` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_A02` | TBD (not in spec_auth_matrix) | `A04`, `A10` | N/A | N/A | N/A | N/A | N/A | Deferred | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_A03` | Should Have / Rot Candidate | `A12` | N/A | ⚠️ | N/A | N/A | N/A | ⚠️ | - |
| `SPEC_A04` | TBD (not in spec_auth_matrix) | `A06` | N/A | N/A | ✅ | N/A | N/A | ✅ | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_A05` | Required | `A13` | N/A | N/A | N/A | N/A | N/A | Deferred | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_A06` | TBD (not in spec_auth_matrix) | `A08` | N/A | N/A | ✅ | N/A | N/A | ✅ | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_A07` | TBD (not in spec_auth_matrix) | `A14` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_A08` | TBD (not in spec_auth_matrix) | `A15` | N/A | N/A | N/A | N/A | N/A | Deferred | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_A09` | Should Have | `A17` | N/A | N/A | N/A | N/A | N/A | Deferred | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_L01` | TBD (not in spec_auth_matrix) | `L13` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L02` | Required | `L02`, `L07` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L03` | TBD (not in spec_auth_matrix) | `L08` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L04` | Bloat? | `L09` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L05` | TBD (not in spec_auth_matrix) | `L10` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L06` | Should Have | `L11` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L07` | TBD (not in spec_auth_matrix) | `G01` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L08` | Should Have | `L12` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_L09` | TBD (not in spec_auth_matrix) | - | N/A | N/A | N/A | N/A | N/A | Deferred | No scenario anchor in spec heading. |
| `SPEC_P01` | Required | `P15` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P02` | Required | `P16` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P03` | Required | `P08` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P04` | Required | `P04`, `P09`, `P10`, `P18` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P05` | Bloat Candidate | `P10`, `P11` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P06` | Rot Candidate | `P12` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P07` | Should Have | `P13` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P08` | Rot Candidate | `P14`, `P42` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_P09` | Required | `P17`, `P25` | N/A | N/A | ✅ | ⚠️ | N/A | ⚠️ | - |
| `SPEC_P10` | Should Have | `P27`, `P28`, `P29`, `P30`, `P33`, `P34`, `P41` | N/A | N/A | ✅ | ✅ | N/A | ✅ | - |

## System Specs

| Spec | Auth class | Scenario anchor(s) | stub | ros | PROD | PY | rclrs | Overall | Notes |
|---|---|---|---|---|---|---|---|---|---|
| `SPEC_C01` | TBD (not in spec_auth_matrix) | `S20` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_C02` | Required | `S01`, `S02` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_C03` | TBD (not in spec_auth_matrix) | `S03` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_C04` | Required | `S04` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_C05` | Bloat? | `S05` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_C06` | TBD (not in spec_auth_matrix) | `S06` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_C07` | TBD (not in spec_auth_matrix) | `S14` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_E01` | Required | `S07` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_E02` | TBD (not in spec_auth_matrix) | `S08` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_E03` | Required | `S09` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_E04` | TBD (not in spec_auth_matrix) | - | N/A | N/A | N/A | N/A | N/A | Deferred | No scenario anchor in spec heading. |
| `SPEC_E05` | TBD (not in spec_auth_matrix) | `S10` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_E06` | TBD (not in spec_auth_matrix) | `S11` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_E07` | TBD (not in spec_auth_matrix) | - | N/A | N/A | N/A | N/A | N/A | Deferred | No scenario anchor in spec heading. |
| `SPEC_ECO01` | TBD (not in spec_auth_matrix) | `S12` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_ECO02` | TBD (not in spec_auth_matrix) | `S13` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_ECO03` | TBD (not in spec_auth_matrix) | `A12` | N/A | ⚠️ | N/A | N/A | N/A | ⚠️ | Derived/projection spec; evidence may be inherited from originating obligation. |
| `SPEC_ECO04` | TBD (not in spec_auth_matrix) | `S14` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_ECO05` | TBD (not in spec_auth_matrix) | `H00` | ✅ | ✅ | ✅ | ✅ | N/A | ✅ | - |
| `SPEC_SYS01` | Required | `S15` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_SYS02` | TBD (not in spec_auth_matrix) | `S16` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_SYS03` | TBD (not in spec_auth_matrix) | `S17` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_SYS04` | TBD (not in spec_auth_matrix) | `S18` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_SYS05` | Rot Candidate | `S19` | N/A | N/A | N/A | N/A | N/A | Deferred | - |
| `SPEC_SYS06` | TBD (not in spec_auth_matrix) | `A17` | N/A | N/A | N/A | N/A | N/A | Deferred | - |

Evidence basis:
- `docs/provenance/scenario/scenario_execution_matrix.md`
- `docs/spec/spec_execution_matrix.md`
- `docs/provenance/divergence_records/divergence_coverage_map.md`
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/stdout.log` (infra failure context)
