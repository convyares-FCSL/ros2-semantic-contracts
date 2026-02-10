# Scenario Authorization Matrix (Parameters)

Last updated: 2026-02-10

Scope:
- Full parameter scenario authorization/disposition matrix.
- Synchronized with:
  - `docs/spec/spec_execution_matrix.md`
  - `docs/spec/spec_scenario_matrix.md`

Legend:
- Auth class: `Required`, `Should Have`, `Bloat Candidate`, `Rot Candidate`, `Probe-only`, `Discovery`.
- Closure use:
  - `Closure`: counts as formal closure evidence for retained points.
  - `Control`: required counterfactual/control evidence in a closure pair.
  - `Probe-only`: retained for signature tracking, not closure-gating.
  - `Discovery`: informational probe, not closure-gating.

| Scenario | Validates | Layer | Auth class | Closure use | Disposition |
|---|---|---|---|---|---|
| `P01` | `SPEC_PC01`, `SPEC_PC02` | Core | Required | Closure | Keep |
| `P02` | `SPEC_PC03` | Core | Required | Closure | Keep |
| `P03` | `SPEC_PC04` | Core | Required | Closure | Keep |
| `P04` | `SPEC_PC05`, `SPEC_P04` | Core | Required | Closure | Keep |
| `P05` | `SPEC_PC05` | Core | Required | Closure | Keep |
| `P06` | `SPEC_PC06` | Core | Required | Closure | Keep |
| `P07` | `SPEC_PC07` (retired) | Core | Probe-only | Probe-only | Keep as signature probe |
| `P08` | `SPEC_PC03`, `SPEC_P03` | Core | Required | Closure | Keep |
| `P09` | `SPEC_P04` | Core | Required | Closure | Keep |
| `P10` | `SPEC_P04`, `SPEC_P05.P1` | Core | Required / Bloat Candidate | Closure | Keep |
| `P11` | `SPEC_P05.P1` | Core | Bloat Candidate (retained point) | Closure | Keep |
| `P12` | `SPEC_P06` | Core | Rot Candidate (rewritten) | Closure | Keep |
| `P13` | `SPEC_P07` | Tooling | Should Have | Closure | Keep |
| `P14` | `SPEC_PC08`, `SPEC_P08` | Policy | Rot Candidate (rewritten) | Closure | Keep |
| `P15` | `SPEC_P01` | Global | Required | Closure | Keep (proxy/deferred direct topology primitive) |
| `P16` | `SPEC_PC06`, `SPEC_P02` | Core | Required | Closure | Keep |
| `P17` | `SPEC_PC13`, `SPEC_P09` | Global | Required | Closure | Keep |
| `P18` | `SPEC_PC05`, `SPEC_P04` | Global | Required | Closure | Keep |
| `P19` | Discovery (pre-set removal semantics) | Global | Discovery | Discovery | Keep informational, not closure-gating |
| `P20` | `SPEC_P05.P2` (retired) | Global | Probe-only | Probe-only | Keep as divergence/signature probe |
| `P21` | `SPEC_PC09` | Core | Should Have | Closure | Keep |
| `P22` | `SPEC_PC10` | Core | Should Have | Closure | Keep |
| `P23` | `SPEC_PC11` | Core | Should Have | Closure | Keep |
| `P24` | `SPEC_PC12` | Core | Required (failing) | Closure | Keep as active divergence anchor |
| `P25` | `SPEC_PC14`, `SPEC_P09` | Core | Should Have / Required | Closure | Keep as active divergence anchor |
| `P26` | `SPEC_PC15` | Core | Required | Closure | Keep |
| `P27` | `SPEC_PC16`, `SPEC_P09` | Core | Should Have / Required | Closure | Keep |
| `P28` | `SPEC_PC16`, `SPEC_P04` | Core | Should Have / Required | Closure | Keep |
| `P29` | `SPEC_PC16`, `SPEC_P10` | Core | Should Have | Closure | Keep |
| `P30` | `SPEC_PC16`, `SPEC_P10` | Global | Should Have | Closure | Keep |
| `P31` | `SPEC_PC11` | Core | Should Have | Closure | Keep as active divergence anchor |
| `P32` | `SPEC_P05.P2` (retired) | Global | Probe-only | Probe-only | Keep as startup signature probe |
| `P33` | `SPEC_PC16` | Core | Should Have | Closure | Keep |
| `P34` | `SPEC_PC16`, `SPEC_P10` | Core | Should Have | Closure | Keep |
| `P35` | `SPEC_PC14.P2` | Core | Should Have | Control | Keep as required control for `P25` |
| `P36` | `SPEC_PC17` | Core | Required | Closure | Keep |
| `P37` | `SPEC_PC17` | Core | Required | Closure | Keep |
| `P38` | `SPEC_PC06.P2`, `SPEC_P02.P2` | Core | Required | Closure | Keep (currently happy-only for mapped points) |
| `P41` | `SPEC_PC16.P3`, `SPEC_P10.P2` | Global | Should Have | Control | Keep as required control for `P29` |
| `P42` | `SPEC_PC08`, `SPEC_P08` | Policy | Rot Candidate (rewritten) | Closure | Keep |

## Parameter authorization summary

- Closure-gating scenarios: `36`
- Control scenarios: `2` (`P35`, `P41`)
- Probe-only scenarios: `3` (`P07`, `P20`, `P32`)
- Discovery-only scenario: `1` (`P19`)

Open divergence anchors remain:
- `P24` (`SPEC_PC12`)
- `P25` (`SPEC_PC14.P1` / `SPEC_P09.P2`)
- `P31` (`SPEC_PC11.P3`)
