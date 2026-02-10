# Spec Execution Matrix Plan and Tracker

Status: Draft  
Last updated: 2026-02-10  
Scope: Spec quality audit + obligation-point coverage tracking

## 1) Why this exists

The current review artifacts are useful inputs, but they are not the final formal source:
- `docs/spec/spec_auth_matrix.md`
- `docs/provenance/scenario/scenario_auth_matix.md`
- `docs/provenance/scenario/scenaios_missing.md`

This plan defines how to replace them with a formal spec-level execution view:
- target document: `docs/spec/spec_execution_matrix.md`

## 2) Rule corrections (binding for this effort)

1. Coverage unit is not "one scenario per spec".
2. Coverage unit is "one scenario mapping per normative spec point".
3. A single scenario may satisfy multiple spec points.
4. A single spec point may require multiple scenarios.
5. For required points, coverage is incomplete without both:
- positive/happy-path evidence
- negative-control evidence
6. Exception handling:
- if a negative control is non-automatable, mark `Deferred` with explicit reason and required primitive.

## 3) Definitions

- `Spec`: a contract ID (example: `SPEC_PC14`).
- `Spec point`: one independently testable normative obligation within a spec.
- `Positive path`: demonstrates allowed behavior works.
- `Negative control`: demonstrates forbidden or counterfactual behavior is rejected or constrained.
- `Shared scenario`: one scenario mapped to multiple points.
- `Control pair`: two scenarios used together to validate one point (example: `P25` + `P35` for `SPEC_PC14`).

## 4) Matrix model for `spec_execution_matrix.md`

Each row is one `Spec Point`, not one `Spec`.

Proposed columns:
- `Spec`
- `Point ID`
- `Point summary`
- `spec_auth status` (`Required`, `Should Have`, `Bloat Candidate`, `Rot Candidate`)
- `Happy scenario(s)`
- `Negative-control scenario(s)`
- `Jazzy cpp`
- `Jazzy py`
- `Humble cpp`
- `Humble py`
- `Coverage state`
- `Evidence`
- `Notes`

Legend proposal:
- backend status: `✅` pass, `❌` fail, `⚠️` mixed/divergent, `N/A` not implemented
- coverage state:
- `Complete` = happy + negative controls mapped and judged
- `Happy-only` = no negative control yet
- `Negative-only` = no happy-path evidence yet
- `Unmapped` = no scenario mapped
- `Deferred` = blocked by missing primitive/manual-only assertion

## 5) Immediate seeded examples (to lock model)

These are seed rows for the formal matrix and should be carried forward.

| Spec | Point ID | Point summary | Happy scenario(s) | Negative-control scenario(s) | Why this matters |
|---|---|---|---|---|---|
| `SPEC_PC14` | `SPEC_PC14.P1` | Non-atomic set path may expose partial visibility during in-flight updates. | `P25` | `P35` (atomic control: partial visibility must not appear) | Canonical example where one spec point needs a control pair. |
| `SPEC_PC16` | `SPEC_PC16.P1` | Standard batch applies earlier successful writes even if later write fails. | `P27` | `P28` (atomic rollback control) | Distinguishes non-atomic semantics from atomic semantics. |
| `SPEC_PC17` | `SPEC_PC17.P1` | Integer descriptor range is enforced on set. | `P36` (in-range + out-of-range in one scenario) | `P36` (same scenario includes reject path) | One scenario can carry both happy and negative assertions when explicit. |
| `SPEC_PC17` | `SPEC_PC17.P2` | Floating descriptor range is enforced on set. | `P37` (in-range + out-of-range in one scenario) | `P37` (same scenario includes reject path) | Same pattern as integer range. |

## 6) Execution plan

### Phase 0: Governance lock
- Confirm no new class layers are introduced.
- Use existing `spec_auth_matrix` status bands as-is:
- `Required`
- `Should Have`
- `Bloat Candidate`
- `Rot Candidate`
- Confirm that the three review artifacts remain input-only during transition.

### Phase 1: Spec-point inventory
- Extract all normative points from:
- `docs/spec/core/*.md`
- `docs/spec/global/*.md`
- `docs/spec/system/*.md`
- Assign stable point IDs as `SPEC_XXX.Pn`.
- Mark each point with `spec_auth` status and rationale.

### Phase 2: Scenario mapping by polarity
- Map each point to existing scenario(s).
- Mark mapping role:
- `Happy`
- `Negative`
- `Control`
- Mark scenario-sharing explicitly.
- Mark unmapped points and one-sided mappings.

### Phase 3: Gap backlog generation
- Generate backlog from unmapped or one-sided points.
- Prioritize in this order:
- `Required` points missing negative controls
- `Required` points missing happy paths
- `Should Have` points missing one side
- Define scenario contracts for each gap:
- stimulus
- observable pass/fail
- backend judgeability constraints

### Phase 4: Formal publish
- Create `docs/spec/spec_execution_matrix.md` with the agreed schema.
- Populate initial rows for all parameter points first, then A/L/S/G/H.
- Cross-link to:
- `docs/spec/spec_scenario_matrix.md`
- `docs/provenance/scenario/scenario_execution_matrix.md`
- `docs/provenance/divergence_records/divergence_coverage_map.md`

### Phase 5: Replace temporary review artifacts
- Rewrite and/or retire:
- `docs/spec/spec_auth_matrix.md`
- `docs/provenance/scenario/scenario_auth_matix.md`
- `docs/provenance/scenario/scenaios_missing.md`
- Preserve useful rationale by migrating it into:
- matrix `Notes`
- spec point classification rationale
- explicit deferred/backlog sections

## 7) Phase 1 execution output (Parameter specs, initial)

Status labels below are taken directly from `docs/spec/spec_auth_matrix.md`:
- `Required`
- `Should Have`
- `Bloat Candidate`
- `Rot Candidate`

Gate order used in this section:
1. Spec worthiness/validity first
2. Scenario rigor second
3. Mapping and gap actions third

### 7.0 Worthiness and validity verdicts (Parameter initial)

Verdict meanings:
- `Keep` = valid and worth carrying as a formal spec
- `Rewrite` = concept worth keeping, but wording/authority/shape is currently invalid or conflicting
- `Split` = one spec contains multiple obligations that should be separated into explicit points
- `Retire` = remove from formal spec set

#### Core (`SPEC_PC*`)

| Spec | Current status (`spec_auth_matrix`) | Worthiness | Validity | Decision | Why |
|---|---|---|---|---|---|
| `SPEC_PC01` | Required | High | Validated | Keep (Required) | Core identity invariant with direct evidence. |
| `SPEC_PC02` | Required | High | Validated (bounded) | Keep (Required) | Necessary state model with direct declared/strict/undeclared-allowed evidence; branch-transition edge cases remain bounded. |
| `SPEC_PC03` | Required | High | Validated | Keep (Required) | Core type-safety contract with positive+negative evidence. |
| `SPEC_PC04` | Required | High | Validated | Keep (Required) | Deterministic declaration behavior is essential and testable. |
| `SPEC_PC05` | Required | High | Validated | Keep (Required) | Atomicity and read-only rejection are hard safety invariants. |
| `SPEC_PC06` | Required | High | Partially validated | Keep (Required) | Strict-mode path covered; undeclared-allowed path now has executable coverage, but no explicit negative/control pair yet. |
| `SPEC_PC07` | Bloat Candidate | Low-Med | Weak | Retire | Deterministic change-record/cardinality semantics lack upstream authority and are scheduler/middleware sensitive; removed in `D-P2-01`. |
| `SPEC_PC08` | Rot Candidate | Med | Rewritten (bounded) | Keep (Rot Candidate, rewritten) | Reframed to executable deletion safety points (`P14`, `P42`) without requiring delete-event emission. |
| `SPEC_PC09` | Should Have | Med-High | Validated (baseline) | Keep (Should Have) | Useful tooling invariant; baseline-backed. |
| `SPEC_PC10` | Should Have | High | Validated (bounded) | Keep (Should Have) | Explicit descriptor-typed declaration and mismatch-rejection behavior are now both directly exercised. |
| `SPEC_PC11` | Should Have | High | Validated + divergence | Keep (Should Have) | Strong real-world utility; explicit Jazzy `py` regression captured. |
| `SPEC_PC12` | Required | High | Valid (IDL authority) | Keep (Required) | Protocol conformance point; failing baseline is divergence, not spec invalidity. |
| `SPEC_PC13` | Required | High | Validated | Keep (Required) | Distinguishes non-atomic semantics from atomic semantics. |
| `SPEC_PC14` | Should Have | High | Probe-scoped valid | Keep (Should Have) | Important observer-safety profile; explicitly bounded to probe shape. |
| `SPEC_PC15` | Required | Med-High | Validated (bounded scope) | Keep (Required) | Important anti-corruption behavior on tested scalar paths. |
| `SPEC_PC16` | Should Have | High | Validated (bounded) | Keep (Should Have) | Transactional profile and callback-time visibility now include a positive/control pair (`P29` + `P41`). |
| `SPEC_PC17` | Required | High | Validated | Keep (Required) | Safety-critical range enforcement with strong evidence. |

#### Global (`SPEC_P*`)

| Spec | Current status (`spec_auth_matrix`) | Worthiness | Validity | Decision | Why |
|---|---|---|---|---|---|
| `SPEC_P01` | Required | High | Validated (happy path) | Keep (Required) | Interface conformance remains foundational. |
| `SPEC_P02` | Required | High | Partially validated | Keep (Required) | Strict mode covered; undeclared-allowed mode now has initial executable path via `P38`. |
| `SPEC_P03` | Required | Med | Rewritten | Keep (Required, bounded) | Rewritten to avoid conformance conflict; `SPEC_PC12` remains sole 1:1 authority. |
| `SPEC_P04` | Required | High | Validated | Keep (Required) | Atomic observer semantics are contract-critical. |
| `SPEC_P05` | Bloat Candidate | Med | Rewritten (bounded) | Keep (Bloat Candidate, rewritten) | Retained only commit-vs-reject event semantics (`P10`,`P11`); cardinality/startup determinism claims retired from closure. |
| `SPEC_P06` | Rot Candidate | Med | Rewritten (bounded) | Keep (Rot Candidate, rewritten) | Tightened to stable unknown-describe outcome shape + non-mutation guarantee, backed by strengthened `P12`. |
| `SPEC_P07` | Should Have | Med | Validated | Keep (Should Have) | Tooling stability value with direct repeat-query checks. |
| `SPEC_P08` | Rot Candidate | Med | Rewritten (bounded) | Keep (Rot Candidate, rewritten) | Converted to executable optional-delete profile (`P14`,`P42`); event emission moved to documented profile note, not hard closure contract. |
| `SPEC_P09` | Required | High | Validated + divergence | Keep (Required) | Critical distinction between non-atomic and atomic visibility profiles. |
| `SPEC_P10` | Should Have | High | Validated (bounded) | Keep (Should Have) | Commit-gated visibility profile now has both positive and control coverage (`P29` + `P41`). |

### 7.0.1 Comparison against `docs/spec/spec_auth_matrix.md` (disagreements)

No active parameter-level status disagreement remains after the `SPEC_P03` rewrite.

| Spec | `spec_auth_matrix` status | This plan verdict | Status |
|---|---|---|---|
| `SPEC_P03` | Required | Keep (Required, bounded) | Resolved in this cycle (`D-P0-02`) |

### 7.0.2 Scenario rigor findings (priority deltas)

Rigor grades:
- `A` direct, bounded, and includes happy + negative/control
- `B` direct but one side incomplete or control indirect
- `C` proxy/discovery or underspecified for contract strength

| Point | Current scenario support | Rigor | Finding | Action |
|---|---|---|---|---|
| `SPEC_PC06.P2` / `SPEC_P02.P2` | `P38` | C | Initial happy-path now exists; explicit negative/control coverage still missing. | Add negative/control follow-up in next cycle if retained as closure-critical. |
| `SPEC_PC10.P2` | `P22` | B | Resolved (`D-P1-01`): `P22` now exercises descriptor-typed declaration plus mismatched-set rejection with value preservation. | Keep declare-time mismatch-precedence branch as optional follow-up, not closure-blocking. |
| `SPEC_PC16.P3` / `SPEC_P10.P2` | `P29`,`P41` | B | Resolved (`D-P1-02`): callback old-value claim now has an explicit control pair. | Keep control pair (`P29` + `P41`) as required evidence for this point. |
| `SPEC_PC07.P1` | `P07`,`P20`,`P32` | C | Resolved (`D-P2-01`): deterministic change-record/cardinality clause retired from closure due weak authority and probe-sensitive behavior. | Keep `P07`,`P20`,`P32` as probe-only signatures; no retained closure point. |
| `SPEC_PC08.P2` | `P42` | B | Resolved (`D-P2-03`): unknown-delete safety is now directly exercised as structured `NotDeclared` + no-corruption path. | Keep `P42` in retained delete-safety evidence pair (`P14` + `P42`). |
| `SPEC_P01.P1` | `P15` | B | Proxy path now includes bounded negatives; direct graph introspection still unavailable. | Keep explicit deferred note for direct topology primitive. |
| `SPEC_P05.P2` | `P20`,`P32` | C | Resolved (`D-P2-01`): startup/cardinality determinism clause retired from closure; retained only as probe signature material. | Keep `P20`,`P32` as probe-only scenarios. |
| `SPEC_P06.P1` | `P12` | B | Resolved (`D-P2-02`): rewritten to stable unknown-describe outcome shape with non-mutation guarantee, and scenario tightened. | Keep bounded clause with `P12` as direct evidence. |
| `SPEC_P08.P1` | `P14`,`P42` | B | Resolved (`D-P2-03`): optional-delete profile now mapped to executable declared-delete + unknown-delete-safety paths. | Keep rewritten `SPEC_P08` with `P14`+`P42`; retire event-emission point from closure. |

Full rigor coverage for every parameter spec-point is recorded in Sections `7.1` and `7.2` via the `Rigor` column.

### 7.1 Core parameter points (`SPEC_PC*`)

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

### 7.2 Global parameter points (`SPEC_P*`)

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

### 7.3 Priority delta execution (Parameter, production order)

Scope rule for this section:
- Only closure-critical deltas are scheduled here.
- Work order is strict: `P0 Required` -> `P1 Should Have` -> `P2 Bloat/Rot`.

#### P0 Required deltas (execute first)

| Delta ID | Targets | Work type | Deliverable | Acceptance gate | Status |
|---|---|---|---|---|---|
| `D-P0-01` | `SPEC_PC06.P2`, `SPEC_P02.P2` | Add scenario | `P38` (new) for undeclared-allowed implicit declaration path | Point state moves from `Unmapped` to at least `Happy-only`; rigor `C` -> `B` or better. | Implemented (Happy-only reached) |
| `D-P0-02` | `SPEC_P03.P2` vs `SPEC_PC12.*` | Spec rewrite + mapping cleanup | Rewrite `SPEC_P03` wording to remove conformance conflict while preserving observed-profile notes | Conflict flag removed from matrix row; `SPEC_PC12` remains authoritative for 1:1 conformance. | Implemented (conflict resolved) |
| `D-P0-03` | `SPEC_P01.P1` | Scenario update | Update `P15` with stricter interface conformity assertions (exact service/topic/type + bounded negative checks) | Point moves from `Happy-only` to `Complete` or explicit `Deferred` with primitive needed. | Implemented (explicit `Deferred`) |

#### P1 Should Have deltas (after P0 complete)

| Delta ID | Targets | Work type | Deliverable | Acceptance gate | Status |
|---|---|---|---|---|---|
| `D-P1-01` | `SPEC_PC10.P2` | Add/update scenario | Extend `P22` or add `P39` mismatch branch for descriptor-type conflict | Point moves from `Unmapped` to `Complete`/`Happy-only` with rigor >= `B`. | Implemented (`P22` strengthened; point now `Complete`, rigor `A`) |
| `D-P1-02` | `SPEC_PC16.P3`, `SPEC_P10.P2` | Add control scenario | `P41` (new) to prove no premature external visibility before commit | Both points gain explicit negative/control coverage; rigor `C` -> `B` or better. | Implemented (`P41` added; both points now `Complete`, rigor `B`) |

#### P2 Bloat/Rot deltas (execute after P0/P1)

| Delta ID | Targets | Work type | Deliverable | Status |
|---|---|---|---|---|
| `D-P2-01` | `SPEC_PC07.P1`, `SPEC_P05.P2` | Rewrite + retire | Retire deterministic/cardinality closure points; keep `P07`,`P20`,`P32` as probe-only signatures. | Implemented (both points retired from closure) |
| `D-P2-02` | `SPEC_P06.P1` | Rewrite | Tighten unknown-describe contract to stable outcome shape + non-mutation; strengthen `P12`. | Implemented (`SPEC_P06` rewritten, `P12` tightened) |
| `D-P2-03` | `SPEC_PC08.*`, `SPEC_P08.*` | Rewrite + scenario add | Reframe optional deletion into executable safety points and add unknown-delete scenario `P42`. | Implemented (`SPEC_PC08`/`SPEC_P08` rewritten, `P42` added) |

#### Out of closure scope (current cycle)

| Scenario | Decision | Rationale |
|---|---|---|
| `P07` | Exclude from closure set | Ordering signature probe retained after `SPEC_PC07` retirement. |
| `P19` | Exclude from closure set | Discovery-only extension; not mapped to retained closure-critical point. |
| `P20` | Exclude from closure set | Issue-signature/divergence probe; not sufficient as standalone normative closure evidence. |
| `P32` | Exclude from closure set | Startup event signature probe retained after `SPEC_P05.P2` retirement. |

## 8) Tracker

Use this checklist as the current working tracker.

- [x] Phase 0 policy agreed (gate order recorded for parameter pass)
- [x] Spec-point ID convention finalized
- [x] Parameter spec points inventoried (`SPEC_PC*`, `SPEC_P*`) (initial pass)
- [ ] A/L/S/G/H spec points inventoried
- [x] Existing scenarios polarity-tagged (`Happy`, `Negative`, `Control`) (parameter initial pass)
- [x] Parameter worthiness/validity gate run (initial pass)
- [x] Parameter scenario-rigor gate run (initial pass)
- [x] Required-point priority deltas defined (`D-P0-*`)
- [x] `D-P0-01` implemented (`P38`)
- [x] `D-P0-02` implemented (resolve `SPEC_P03` vs `SPEC_PC12`)
- [x] `D-P0-03` implemented (`P15` hardening + explicit deferred topology note)
- [x] P0 acceptance gates passed
- [x] P1 delta queue started (`D-P1-*`)
- [x] `D-P1-01` implemented (`P22` mismatch enforcement branch)
- [x] `D-P1-02` implemented (`P41` premature-visibility control)
- [x] P1 acceptance gates passed
- [x] `D-P2-01` implemented (`SPEC_PC07.P1` + `SPEC_P05.P2` retired; probe retention only)
- [x] `D-P2-02` implemented (`SPEC_P06.P1` rewritten + `P12` tightened)
- [x] `D-P2-03` implemented (`SPEC_PC08`/`SPEC_P08` rewritten + `P42` added)
- [x] P2 acceptance gates passed
- [x] Draft `docs/spec/spec_execution_matrix.md` created
- [x] Parameter section populated with evidence
- [x] Non-parameter sections populated
- [x] Transition plan approved for replacing 3 review artifacts

## 9) Acceptance criteria

This effort is complete when all are true:
- Every required spec point has explicit polarity coverage state.
- Every required spec point is either:
- `Complete`, or
- `Deferred` with concrete blocker and planned primitive.
- Shared-scenario and multi-scenario mappings are explicit and reviewable.
- `spec_execution_matrix.md` can answer "what spec points are proven, where, and by which controls" without reading review artifacts.
