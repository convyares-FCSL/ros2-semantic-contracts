# Spec Execution Matrix

Status: Draft v1 (transition publish)  
Last updated: 2026-02-10

Purpose:
- This is the formal spec execution matrix referenced by `docs/_work/spec_execution_matrix_plan.md`.
- It is the replacement target for review-only artifacts:
  - `docs/spec/spec_auth_matrix.md`
  - `docs/provenance/scenario/scenario_auth_matix.md`
  - `docs/provenance/scenario/scenaios_missing.md`

## 1) Matrix Conventions

Coverage state legend:
- `Complete`: happy + negative/control evidence mapped and judged.
- `Happy-only`: positive path mapped; no explicit negative/control yet.
- `Negative-only`: negative/control mapped; no explicit happy path yet.
- `Unmapped`: no scenario mapping is currently executable.
- `Deferred`: blocked by missing primitive, manual-only assertion, or infra break.
- `Retired`: point intentionally removed from closure.

Execution legend:
- `✅` pass
- `❌` fail
- `⚠️` mixed/divergent or infra caution
- `N/A` not implemented or not executed

Primary evidence sources:
- `docs/provenance/scenario/scenario_execution_matrix.md`
- `docs/spec/spec_scenario_matrix.md`
- `docs/provenance/divergence_records/divergence_coverage_map.md`
- `evidence/reliability/*/run_01/report.json`

## 2) Parameter Spec-Point Matrix (v1)

This section is the current authoritative point-level matrix for parameters.

### 2.1 Core Parameter Points (`SPEC_PC*`)

| Spec | Point ID | `spec_auth` status | Obligation point | Happy | Negative / control | State | Rigor |
|---|---|---|---|---|---|---|---|
| `SPEC_PC01` | `SPEC_PC01.P1` | Required | Parameter names are case-sensitive identities. | `P01` | `P01` | Complete | A |
| `SPEC_PC01` | `SPEC_PC01.P2` | Required | Core must not fabricate undeclared parameters. | `P01` | `P01` | Complete | A |
| `SPEC_PC02` | `SPEC_PC02.P1` | Required | Existence-state model (Declared / Undeclared-Allowed / Unknown) is coherent. | `P01`,`P16`,`P38` | `P06` | Complete | B |
| `SPEC_PC03` | `SPEC_PC03.P1` | Required | Static typing rejects incompatible set requests. | `P02` | `P02`,`P26` | Complete | A |
| `SPEC_PC03` | `SPEC_PC03.P2` | Required | Dynamic typing allows explicit type transition on set. | `P02`,`P26` | `P26` | Complete | A |
| `SPEC_PC04` | `SPEC_PC04.P1` | Required | Redeclaration of declared key fails deterministically. | `P03` | `P03` | Complete | A |
| `SPEC_PC05` | `SPEC_PC05.P1` | Required | Atomic batch is all-or-nothing. | `P04`,`P18`,`P28` | `P04`,`P18`,`P28` | Complete | A |
| `SPEC_PC05` | `SPEC_PC05.P2` | Required | Read-only descriptors reject value mutation attempts. | `P05` | `P05` | Complete | A |
| `SPEC_PC06` | `SPEC_PC06.P1` | Required | In strict mode, undeclared set fails deterministically. | `P16` | `P06`,`P16` | Complete | A |
| `SPEC_PC06` | `SPEC_PC06.P2` | Required | In undeclared-allowed mode, unknown set implicitly declares. | `P38` | - | Happy-only | C |
| `SPEC_PC07` | `SPEC_PC07.P1` | Bloat Candidate | Deterministic change-record/cardinality clause (retired in `D-P2-01`). | - | - | Retired | - |
| `SPEC_PC08` | `SPEC_PC08.P1` | Rot Candidate | If deletion is supported, declared-key undeclare removes state safely. | `P14` | `P42` | Complete | B |
| `SPEC_PC08` | `SPEC_PC08.P2` | Rot Candidate | Unknown-key undeclare is safe fail/no-op and preserves unrelated state. | `P42` | `P42` | Complete | B |
| `SPEC_PC09` | `SPEC_PC09.P1` | Should Have | Descriptor `type` tracks current dynamic value type after commit. | `P21` | - | Happy-only | C |
| `SPEC_PC10` | `SPEC_PC10.P1` | Should Have | Explicit descriptor type is preserved/enforced on declaration. | `P22` | - | Happy-only | B |
| `SPEC_PC10` | `SPEC_PC10.P2` | Should Have | For explicitly descriptor-typed declarations, mismatched set attempts are rejected and prior committed value is preserved. | `P22` | `P22` | Complete | A |
| `SPEC_PC11` | `SPEC_PC11.P1` | Should Have | Explicit typed declaration without default is permitted. | `P23` | `P23` | Complete | A |
| `SPEC_PC11` | `SPEC_PC11.P2` | Should Have | No-default declaration reads as `NOT_SET` until initialized. | `P23` | `P23` | Complete | A |
| `SPEC_PC11` | `SPEC_PC11.P3` | Should Have | Startup override binds typed no-default declaration without throw. | `P31` | `P31` | Divergent (`Jazzy py`) | B |
| `SPEC_PC12` | `SPEC_PC12.P1` | Required (Failing) | `get_parameter_types` response length equals request length (1:1). | `P24` | `P24` | Divergent (all measured cells fail) | A |
| `SPEC_PC12` | `SPEC_PC12.P2` | Required (Failing) | Undeclared names map to explicit `NOT_SET` entries, not collapse. | `P24` | `P24` | Divergent (all measured cells fail) | A |
| `SPEC_PC13` | `SPEC_PC13.P1` | Required | Non-atomic batch may partially apply earlier successful keys. | `P17`,`P27`,`P33` | `P28`,`P34` | Complete | A |
| `SPEC_PC14` | `SPEC_PC14.P1` | Should Have | Non-atomic concurrent probe may expose mixed cross-key snapshots. | `P25` | `P35` | Divergent (`py` asymmetry) | B |
| `SPEC_PC14` | `SPEC_PC14.P2` | Should Have | Atomic control probe should not expose mixed cross-key snapshots. | `P35` | `P25` | Complete | B |
| `SPEC_PC15` | `SPEC_PC15.P1` | Required | Tested static scalar cross-type sets are rejected (no implicit coercion). | `P26` | `P26` | Complete | A |
| `SPEC_PC15` | `SPEC_PC15.P2` | Required | Rejected set preserves prior committed value/type. | `P26` | `P26` | Complete | A |
| `SPEC_PC16` | `SPEC_PC16.P1` | Should Have | Standard batch profile keeps earlier successful keys when later key fails. | `P27`,`P33` | `P28`,`P34` | Complete | A |
| `SPEC_PC16` | `SPEC_PC16.P2` | Should Have | Atomic profile rejects mixed-validity batch as one transaction. | `P28`,`P34` | `P27`,`P33` | Complete | A |
| `SPEC_PC16` | `SPEC_PC16.P3` | Should Have | Callback-time storage reads return old committed value before commit. | `P29` | `P41` | Complete | B |
| `SPEC_PC16` | `SPEC_PC16.P4` | Should Have | External visibility is commit-gated (reject silent, success visible). | `P30` | `P30`,`P34` | Complete | A |
| `SPEC_PC17` | `SPEC_PC17.P1` | Required | Integer descriptor range rejects out-of-range writes and preserves last good value. | `P36` | `P36` | Complete | A |
| `SPEC_PC17` | `SPEC_PC17.P2` | Required | Floating descriptor range rejects out-of-range writes and preserves last good value. | `P37` | `P37` | Complete | A |
| `SPEC_PC17` | `SPEC_PC17.P3` | Required | `describe_parameters` exposes configured numeric range metadata. | `P36`,`P37` | `P36`,`P37` | Complete | A |

### 2.2 Global Parameter Points (`SPEC_P*`)

| Spec | Point ID | `spec_auth` status | Obligation point | Happy | Negative / control | State | Rigor |
|---|---|---|---|---|---|---|---|
| `SPEC_P01` | `SPEC_P01.P1` | Required | Standard parameter services/topic are exposed. | `P15` | `P15` (bounded negatives) | Deferred (direct graph introspection primitive missing) | B |
| `SPEC_P02` | `SPEC_P02.P1` | Required | Declared-only mode rejects undeclared set attempts. | `P16` | `P16` | Complete | A |
| `SPEC_P02` | `SPEC_P02.P2` | Required | Undeclared-allowed mode behavior is supported and documented. | `P38` | - | Happy-only | C |
| `SPEC_P03` | `SPEC_P03.P1` | Required | `get_parameters` unknown key path returns `NOT_SET`-typed values. | `P08` | `P08` | Complete | A |
| `SPEC_P03` | `SPEC_P03.P2` | Required | Undeclared-name `get_parameter_types` behavior is documented as profile-only; 1:1 conformance belongs to `SPEC_PC12`. | `P24` (observed profile note) | - | Deferred (documentation/profile point) | C |
| `SPEC_P04` | `SPEC_P04.P1` | Required | Atomic set requests do not expose partial externally visible state. | `P04`,`P09`,`P18`,`P28` | `P04`,`P18`,`P28` | Complete | A |
| `SPEC_P04` | `SPEC_P04.P2` | Required | Failed atomic set requests emit no change event. | `P10`,`P30` | `P10`,`P30` | Complete | A |
| `SPEC_P05` | `SPEC_P05.P1` | Bloat Candidate | Commit-only event semantics: successful changes are represented; rejected attempts are silent. | `P11` | `P10` | Complete | A |
| `SPEC_P05` | `SPEC_P05.P2` | Bloat Candidate | Event cardinality/startup deterministic-envelope clause (retired in `D-P2-01`). | - | - | Retired | - |
| `SPEC_P06` | `SPEC_P06.P1` | Rot Candidate | Unknown describe profile is stable and does not mutate unrelated declared state. | `P12` | `P12` | Complete | B |
| `SPEC_P07` | `SPEC_P07.P1` | Should Have | `list_parameters` results are complete for prefix/depth query shape. | `P13` | `P13` | Complete | A |
| `SPEC_P07` | `SPEC_P07.P2` | Should Have | `list_parameters` results are stable under repeated identical queries. | `P13` | `P13` | Complete | A |
| `SPEC_P08` | `SPEC_P08.P1` | Rot Candidate | Optional deletion profile: declared-key delete removes state; unknown-key delete is safe/no-corruption. | `P14`,`P42` | `P42` | Complete | B |
| `SPEC_P08` | `SPEC_P08.P2` | Rot Candidate | Delete-event emission hard requirement (retired; now documented profile note only). | - | - | Retired | - |
| `SPEC_P09` | `SPEC_P09.P1` | Required | Non-atomic batch and atomic batch are distinct externally observable profiles. | `P17`,`P27` | `P18`,`P28` | Complete | A |
| `SPEC_P09` | `SPEC_P09.P2` | Required | Concurrent read probes may expose mixed non-atomic visibility. | `P25` | `P35` | Divergent (`py` asymmetry) | B |
| `SPEC_P10` | `SPEC_P10.P1` | Should Have | Visibility is commit-gated: rejection silent, success visible after commit. | `P30`,`P34` | `P30`,`P34` | Complete | A |
| `SPEC_P10` | `SPEC_P10.P2` | Should Have | Callback-time storage remains old value until commit finalization. | `P29` | `P41` | Complete | B |

### 2.3 Parameter Execution Signal (Cross-Distro Focus Rows)

| Spec / point focus | Scenario(s) | Jazzy cpp | Jazzy py | Humble cpp | Humble py | Status |
|---|---|---|---|---|---|---|
| `SPEC_PC12.P1/.P2` | `P24` | ❌ | ❌ | ❌ | ❌ | Open divergence (1:1 contract) |
| `SPEC_PC11.P3` | `P31` | ✅ | ❌ | ✅ | ✅ | Jazzy `rclpy` regression |
| `SPEC_PC14.P1` | `P25` | ✅ | ❌ | ✅ | ❌ | Non-atomic visibility asymmetry |
| `SPEC_PC14.P2` | `P35` | ✅ | ✅ | ✅ | ✅ | Atomic control parity pass |
| Probe-only (`SPEC_P05.P2` retired) | `P32` | ✅ | ✅ | ✅ | ⚠️ | Humble `py` full-bundle refresh blocked by infra (`Node.list_parameters` missing) |

### 2.4 Parameter Evidence

- `docs/provenance/scenario/scenario_execution_matrix.md`
- `docs/spec/spec_scenario_matrix.md`
- `docs/provenance/divergence_records/divergence_coverage_map.md`
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/stdout.log`

## 3) Non-Parameter Sections (Seed Population)

Note:
- These sections are intentionally seeded at one provisional point per spec (`.P1`) to publish a complete formal surface.
- Full non-parameter point decomposition remains open in the tracker (`A/L/S/G/H spec points inventoried`).

### 3.1 Action Seed Matrix

| Spec | Seed point ID | `spec_auth` status | Scenario anchor(s) | Current execution signal | Coverage state | Notes |
|---|---|---|---|---|---|---|
| `SPEC_AC01` | `SPEC_AC01.P1` | Required | `A01`,`A03` | `A01` pass on `PROD`; `A03` unvalidated | Happy-only | Requires explicit negative/control completion. |
| `SPEC_AC02` | `SPEC_AC02.P1` | TBD (not classified in `spec_auth_matrix`) | `A15` | Not executed | Deferred | Scenario note requires adversarial UUID primitive. |
| `SPEC_AC03` | `SPEC_AC03.P1` | Required | `A02` | `A02` pass on `PROD` | Happy-only | Control side not yet automated. |
| `SPEC_AC04` | `SPEC_AC04.P1` | Required | `A04`,`A05` | Not executed | Unmapped | Scenario anchors exist but are not validated. |
| `SPEC_AC05` | `SPEC_AC05.P1` | Required | `A06`,`A07` | `A06` pass on `PROD`; `A07` not executed | Happy-only | Ordering control path pending. |
| `SPEC_AC06` | `SPEC_AC06.P1` | TBD (not classified in `spec_auth_matrix`) | `A08`,`A09` | `A08` pass on `PROD`; `A09` not executed | Happy-only | Resolution-policy branch pending. |
| `SPEC_AC07` | `SPEC_AC07.P1` | TBD (not classified in `spec_auth_matrix`) | `A10`,`A11` | Not executed | Unmapped | Result visibility/singularity not yet exercised. |
| `SPEC_A01` | `SPEC_A01.P1` | TBD (not classified in `spec_auth_matrix`) | `A16` | Not executed | Unmapped | Interface conformity scenario exists but is unrun. |
| `SPEC_A02` | `SPEC_A02.P1` | TBD (not classified in `spec_auth_matrix`) | `A04`,`A10` | Not executed | Unmapped | Derived projection, pending core evidence. |
| `SPEC_A03` | `SPEC_A03.P1` | Should Have | `A12` | `ros` caution/unvalidated | Deferred | Supersession policy remains unvalidated. |
| `SPEC_A04` | `SPEC_A04.P1` | TBD (not classified in `spec_auth_matrix`) | `A06` | `A06` pass on `PROD` | Happy-only | Derived projection from `SPEC_AC05`. |
| `SPEC_A05` | `SPEC_A05.P1` | Required | `A13` | Not executed | Unmapped | Result-retention scenario not run. |
| `SPEC_A06` | `SPEC_A06.P1` | TBD (not classified in `spec_auth_matrix`) | `A08` | `A08` pass on `PROD` | Happy-only | Derived projection from `SPEC_AC06`. |
| `SPEC_A07` | `SPEC_A07.P1` | TBD (not classified in `spec_auth_matrix`) | `A14` | Not executed | Unmapped | Tooling scenario unrun. |
| `SPEC_A08` | `SPEC_A08.P1` | TBD (not classified in `spec_auth_matrix`) | `A15` | Not executed | Deferred | Same UUID-reuse primitive blocker as `SPEC_AC02`. |
| `SPEC_A09` | `SPEC_A09.P1` | Should Have | `A17` | Not executed | Unmapped | Projection from `SPEC_SYS06`; no executed evidence yet. |

### 3.2 Lifecycle and Global-Gate Seed Matrix

| Spec | Seed point ID | `spec_auth` status | Scenario anchor(s) | Current execution signal | Coverage state | Notes |
|---|---|---|---|---|---|---|
| `SPEC_LC01` | `SPEC_LC01.P1` | Required | `L02`,`L07` (derived) | Not executed | Unmapped | Core state inventory has no executed lifecycle scenarios yet. |
| `SPEC_LC02` | `SPEC_LC02.P1` | Required | `L01` | Not executed | Unmapped | Transition graph validation pending. |
| `SPEC_LC03` | `SPEC_LC03.P1` | Required | `L03` | Not executed | Unmapped | Intermediate atomicity not exercised. |
| `SPEC_LC04` | `SPEC_LC04.P1` | TBD (not classified in `spec_auth_matrix`) | `L04`,`L05` | Not executed | Unmapped | Single-active-transition checks not run. |
| `SPEC_LC05` | `SPEC_LC05.P1` | Rot Candidate | `L06` | Not executed | Unmapped | Error-resolution profile still pending execution. |
| `SPEC_L01` | `SPEC_L01.P1` | TBD (not classified in `spec_auth_matrix`) | `L13` | Not executed | Unmapped | Interface conformity path unrun. |
| `SPEC_L02` | `SPEC_L02.P1` | Required | `L02`,`L07` | Not executed | Unmapped | Primary state visibility pending. |
| `SPEC_L03` | `SPEC_L03.P1` | TBD (not classified in `spec_auth_matrix`) | `L08` | Not executed | Unmapped | Deterministic acceptance scenario unrun. |
| `SPEC_L04` | `SPEC_L04.P1` | Bloat? | `L09` | Not executed | Unmapped | Event-emission claim remains unexercised. |
| `SPEC_L05` | `SPEC_L05.P1` | TBD (not classified in `spec_auth_matrix`) | `L10` | Not executed | Unmapped | Shutdown determinism not executed. |
| `SPEC_L06` | `SPEC_L06.P1` | Should Have | `L11` | Not executed | Unmapped | Active-state shutdown path unrun. |
| `SPEC_L07` | `SPEC_L07.P1` | TBD (not classified in `spec_auth_matrix`) | `G01` | Not executed | Unmapped | Global gate scenario not run. |
| `SPEC_L08` | `SPEC_L08.P1` | Should Have | `L12` | Not executed | Unmapped | Negative assertion pending execution. |
| `SPEC_L09` | `SPEC_L09.P1` | TBD (not classified in `spec_auth_matrix`) | - | Not executed | Deferred | Tooling introspection has no isolating scenario yet. |

### 3.3 System, Composition, Executor, Ecosystem, Harness Seed Matrix

| Spec | Seed point ID | `spec_auth` status | Scenario anchor(s) | Current execution signal | Coverage state | Notes |
|---|---|---|---|---|---|---|
| `SPEC_SYS01` | `SPEC_SYS01.P1` | Required | `S15` | Not executed | Unmapped | Ownership/lifetime scenario unrun. |
| `SPEC_SYS02` | `SPEC_SYS02.P1` | TBD (not classified in `spec_auth_matrix`) | `S16` | Not executed | Unmapped | Bringup/teardown determinism unrun. |
| `SPEC_SYS03` | `SPEC_SYS03.P1` | TBD (not classified in `spec_auth_matrix`) | `S17` | Not executed | Deferred | Scenario marked manual/deferred in matrix. |
| `SPEC_SYS04` | `SPEC_SYS04.P1` | TBD (not classified in `spec_auth_matrix`) | `S18` | Not executed | Unmapped | Observable-failure path unrun. |
| `SPEC_SYS05` | `SPEC_SYS05.P1` | Rot Candidate | `S19` | Not executed | Unmapped | Graph hygiene remains unexecuted. |
| `SPEC_SYS06` | `SPEC_SYS06.P1` | TBD (not classified in `spec_auth_matrix`) | `A17` | Not executed | Unmapped | Originating obligation for `SPEC_A09` projection. |
| `SPEC_C01` | `SPEC_C01.P1` | TBD (not classified in `spec_auth_matrix`) | `S20` | Not executed | Unmapped | Composition interface conformity unrun. |
| `SPEC_C02` | `SPEC_C02.P1` | Required | `S01`,`S02` | Not executed | Unmapped | Atomic load semantics unrun. |
| `SPEC_C03` | `SPEC_C03.P1` | TBD (not classified in `spec_auth_matrix`) | `S03` | Not executed | Unmapped | Unload atomic semantics unrun. |
| `SPEC_C04` | `SPEC_C04.P1` | Required | `S04` | Not executed | Unmapped | Node identity contract unrun. |
| `SPEC_C05` | `SPEC_C05.P1` | Bloat? | `S05` | Not executed | Unmapped | Concurrency/determinism scenario unrun. |
| `SPEC_C06` | `SPEC_C06.P1` | TBD (not classified in `spec_auth_matrix`) | `S06` | Not executed | Unmapped | Parameter remap/load contract unrun. |
| `SPEC_C07` | `SPEC_C07.P1` | TBD (not classified in `spec_auth_matrix`) | `S14` | Not executed | Unmapped | CLI/tooling compatibility unrun. |
| `SPEC_E01` | `SPEC_E01.P1` | Required | `S07` | Not executed | Unmapped | Explicit ownership still unexecuted in S-family. |
| `SPEC_E02` | `SPEC_E02.P1` | TBD (not classified in `spec_auth_matrix`) | `S08` | Not executed | Unmapped | Bounded spin modes unrun. |
| `SPEC_E03` | `SPEC_E03.P1` | Required | `S09` | Not executed | Unmapped | Bounded shutdown scenario unrun. |
| `SPEC_E04` | `SPEC_E04.P1` | TBD (not classified in `spec_auth_matrix`) | - | Not executed | Deferred | Documentation/spec assertion, no direct scenario mapping. |
| `SPEC_E05` | `SPEC_E05.P1` | TBD (not classified in `spec_auth_matrix`) | `S10` | Not executed | Unmapped | Failure isolation/observability unrun. |
| `SPEC_E06` | `SPEC_E06.P1` | TBD (not classified in `spec_auth_matrix`) | `S11` | Not executed | Unmapped | Lifecycle composition safety unrun. |
| `SPEC_E07` | `SPEC_E07.P1` | TBD (not classified in `spec_auth_matrix`) | - | Not executed | Deferred | Interface/testability assertion, no direct scenario mapping. |
| `SPEC_ECO01` | `SPEC_ECO01.P1` | TBD (not classified in `spec_auth_matrix`) | `S12` | Not executed | Unmapped | Lifecycle orchestration compatibility unrun. |
| `SPEC_ECO02` | `SPEC_ECO02.P1` | TBD (not classified in `spec_auth_matrix`) | `S13` | Not executed | Deferred | Scenario explicitly deferred (kill-node/adversarial primitive). |
| `SPEC_ECO03` | `SPEC_ECO03.P1` | TBD (not classified in `spec_auth_matrix`) | `A12` | `ros` caution/unvalidated | Deferred | Derived supersession profile remains unvalidated. |
| `SPEC_ECO04` | `SPEC_ECO04.P1` | TBD (not classified in `spec_auth_matrix`) | `S14` | Not executed | Unmapped | Tooling support scenario unrun. |
| `SPEC_ECO05` | `SPEC_ECO05.P1` | TBD (not classified in `spec_auth_matrix`) | `H00` | `H00` pass on `stub`,`ros`,`PROD`,`PY` | Happy-only | Behavioral indistinguishability seeded by harness smoke path. |

### 3.4 Non-Parameter Evidence

- `docs/provenance/scenario/scenario_execution_matrix.md`
- `docs/provenance/scenario/semantics_A.md`
- `docs/provenance/scenario/semantics_L.md`
- `docs/provenance/scenario/semantics_S.md`
- `docs/spec/core/action_core.md`
- `docs/spec/core/lifecycle_core.md`
- `docs/spec/global/action.md`
- `docs/spec/global/lifecycle.md`
- `docs/spec/system/system_contract.md`
- `docs/spec/system/composition.md`
- `docs/spec/system/executor.md`
- `docs/spec/system/ecosystem.md`

## 4) Transition Plan (Approved)

Approval status:
- Approved for transition publishing on 2026-02-10 (this matrix becomes the formal execution source; prior artifacts become input-only during migration).

| Review artifact | Replacement in this matrix | Status |
|---|---|---|
| `docs/spec/spec_auth_matrix.md` | `spec_auth` classification columns in Sections 2 and 3, plus transition notes here. | Approved (transition) |
| `docs/provenance/scenario/scenario_auth_matix.md` | Spec-to-scenario mappings in `Happy` / `Negative-control` columns and seed mappings. | Approved (transition) |
| `docs/provenance/scenario/scenaios_missing.md` | `Coverage state` (`Unmapped` / `Deferred`) and backlog section below. | Approved (transition) |

## 5) Backlog / Deferred Work

- Complete non-parameter point-level decomposition (replace provisional `.P1` seed rows with full point inventory).
- Resolve Humble `py` full-bundle infra break (`Node.list_parameters` missing) before closing probe refresh caution on `P32`.
- Continue closure work on active parameter divergences: `P24`, `P25`, `P31`.
- Decide retirement or rewrite finalization path for high-risk non-parameter candidates (`SPEC_LC05`, `SPEC_L04`, `SPEC_SYS05`, `SPEC_C05`).
