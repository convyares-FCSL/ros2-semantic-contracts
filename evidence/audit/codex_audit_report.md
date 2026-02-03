 ROS 2 Semantic Contracts — Full Audit Report (ROS 2 Ecosystem Alignment)

## Review metadata
- **Local repository**: `/workspace/ros2-semantic-contracts`
- **Local branch**: `work` (no remote configured; default branch not discoverable locally)
- **Latest commit SHA reviewed**: `d88e1801189f88bde9933a727e8fa402377e7bee`
- **Network access**: External web access blocked in this environment (GitHub/ROS docs unreachable), so upstream link validation and direct ROS 2 doc cross-checks are marked **NOT VERIFIED**.

---

## A) Executive summary (overall rating + main blockers)

**Overall rating: C (early-stage conceptual clarity, low empirical credibility).**

### Main blockers to credibility
1. **Traceability is promised but not enforced**: the methodology mandates provenance tags for every normative clause, but most specs include no `VALIDATED/UNVALIDATED/REJECTED` tags, leaving the claims un-auditable against the stated process.
2. **Scenario→Harness→Evidence chain is broken**: scenario semantics exist, but scenario bundles are mostly skeletons, and many bundles violate the scenario schema or rely on event types not defined in the trace schema.
3. **Spec/scenario mismatches**: scenario semantics reference spec IDs that do not exist, while the coverage matrix includes formatting and ID inconsistencies.
4. **Upstream alignment is unverified**: external documentation and REPs are cited but not confirmed (blocked network); the repo does not currently contain offline snapshots or extracted quotes.

---

## B) Strengths (what’s genuinely excellent)
- **Clear epistemology**: the philosophy/methodology articulate how “truth” is established and how uncertainty should be flagged.
- **Layered architecture** (core/global/system) provides strong conceptual separation and is a good mental model for ROS 2 maintainers.
- **Reference semantics engine** (Rust crate) demonstrates intent to make core semantics executable rather than prose-only.
- **Harness contract** explicitly enforces “backend observes; core judges,” which is a strong credibility signal when implemented.

---

## C) Critical issues (must fix before expanding scope)
1. **Provenance tagging is missing** for the majority of MUST/SHALL/FORBIDDEN clauses despite the methodology requiring it.
2. **Scenario bundles violate schema** (missing `spec_id/title/ops/expects` and using non-schema fields like `spec_ref/status/type`).
3. **Trace schema vs scenario expectations are incompatible** (expected event types in scenario bundles do not exist in the trace schema).
4. **Scenario semantics references non-existent spec IDs** (`SPEC_C08`, `SPEC_C10`, `SPEC_C13`, `SPEC_C15`) while the composition spec defines only `SPEC_C01..C07`.
5. **Coverage matrix formatting and mapping is broken**, making it unreliable as evidence of validation.
6. **No CI or automated checks** for link hygiene, schema validity, or traceability consistency.

---

## D) Traceability audit results

### D1) Normative claim inventory (by spec file)
Below is the complete list of normative claims (MUST/MUST NOT/SHALL/FORBIDDEN) extracted per spec section. Each bullet maps to the SPEC identifier in the file heading.

#### docs/spec/core/action_core.md
- **SPEC_AC01**
  - Each goal processed by the engine MUST be uniquely identified.
  - Two distinct goals MUST NOT share the same identifier (UUID) concurrently.
  - The engine MUST treat goals with distinct IDs as semantically independent entities.
- **SPEC_AC02**
  - Goal identity MUST be immutable and non-reusable.
  - Identity MUST remain stable for the entire lifetime of the goal.
  - Once a goal ID reaches a terminal state, that ID MUST NOT be reused for a new goal within the server's execution context.
- **SPEC_AC03**
  - A goal in a terminal state MUST NOT transition to any other state.
- **SPEC_AC04**
  - The core engine MUST enforce a strict state transition graph.
  - Invalid transitions MUST be rejected deterministically.
- **SPEC_AC05**
  - Status updates exposed by the core MUST be monotonically ordered per goal.
  - The sequence of states observed for a single goal MUST strictly follow the transition graph.
  - Ordering MUST NOT depend on wall-clock time.
  - Identical transition sequences MUST produce identical observable orderings.
- **SPEC_AC06**
  - Cancellation intent MUST be explicit and observable (transition to `Canceling`).
  - The core MUST allow resolution to any terminal state from `Canceling`.
- **SPEC_AC07**
  - A goal MUST have at most one terminal result.
  - Result visibility MUST be consistent with the goal state.

#### docs/spec/core/lifecycle_core.md
- **SPEC_LC01**
  - The core engine MUST enforce a state model consisting of primary and auxiliary states.
- **SPEC_LC02**
  - The core engine MUST enforce a strict transition graph.
  - Invalid transitions MUST be rejected deterministically by the engine.
- **SPEC_LC03**
  - The core MUST enter the corresponding transition state internally.
  - Intermediate states MUST NOT be externally exposed as stable lifecycle states.
  - The engine MUST NOT accept new transitions while in an intermediate state.
- **SPEC_LC04**
  - A lifecycle node MUST process at most one transition at a time.
  - Concurrent transition requests MUST be rejected deterministically.
  - Rejection MUST NOT partially apply any transition effects.
- **SPEC_LC05**
  - The node MUST enter `ErrorProcessing`.
  - `ErrorProcessing` MUST resolve deterministically to a primary state.

#### docs/spec/core/parameter_core.md
- **SPEC_PC01**
  - The core MUST NOT fabricate parameters that have not been explicitly set or declared.
- **SPEC_PC02**
  - Unknown parameters MUST NOT appear in list results unless explicitly requested.
- **SPEC_PC03**
  - The core MUST enforce type safety based on the parameter descriptor.
  - Static typing MUST NOT allow type changes after declaration.
- **SPEC_PC04**
  - Declaring an already-declared parameter MUST fail deterministically.
  - Declaration order MUST be preserved in change records.
- **SPEC_PC05**
  - Failed operations MUST NOT partially apply.
  - Read-only parameters MUST reject value changes.
- **SPEC_PC06**
  - If `allow_undeclared_parameters` is false, set attempts for unknown parameters MUST fail deterministically.
- **SPEC_PC07**
  - Each operation MUST produce exactly one change record.
  - Ordering within a record MUST reflect application order.
  - Identical sequences MUST produce identical records.
- **SPEC_PC08**
  - If supported, deletion MUST generate a change record indicating removal.
  - Deleting an unknown parameter MUST fail or be a no-op, but MUST NOT corrupt state.

#### docs/spec/global/action.md
- **SPEC_A01**
  - Action semantics MUST be exposed via standard ROS 2 action services and topics.
- **SPEC_A02**
  - Each goal MUST proceed through a well-defined lifecycle.
  - Terminal outcomes MUST convey semantic meaning to clients.
  - Terminal states MUST be externally observable via status topics.
- **SPEC_A03**
  - Supersession MUST transition to a terminal state indicating supersession.
  - Supersession MUST be distinguishable from failure.
  - Supersession MUST be represented as `CANCELED` (baseline requirement).
- **SPEC_A04**
  - Status and feedback updates MUST be ordered per goal.
  - Clients MUST NOT infer ordering from timestamps.
- **SPEC_A05**
  - Results MUST be retained for a non-zero duration.
  - Retention duration MUST be documented.
- **SPEC_A06**
  - Cancellation intent MUST be observable via the cancel service.
- **SPEC_A07**
  - A compliant implementation MUST support standard ROS 2 tools for action discovery and introspection.
- **SPEC_A08**
  - Goal identifiers MUST NOT be reused within the lifetime of the action server.
  - Once a goal UUID reaches a terminal state, it MUST NOT be used for a subsequent goal.
- **SPEC_A09**
  - Action servers MUST eventually clean up goals from vanished/abandoned clients.
  - Cleanup MUST occur within a bounded observation window.

#### docs/spec/global/lifecycle.md
- **SPEC_L01**
  - Lifecycle semantics MUST be exposed via standard ROS 2 lifecycle services and messages.
- **SPEC_L02**
  - State queries MUST reflect resolved lifecycle state.
  - `get_state` MUST report only primary lifecycle states.
  - Transition states MUST NOT be reported as stable states.
- **SPEC_L03**
  - Valid transition requests MUST be accepted or rejected deterministically.
  - Invalid or busy requests MUST be rejected.
  - Rejection MUST be observable via service response.
  - Busy/invalid rejection emission of `TransitionEvent` MUST be documented.
- **SPEC_L04**
  - Accepted transition attempts MUST emit a `TransitionEvent`.
  - Observers MUST NOT assume identical event semantics across implementations unless validated.
- **SPEC_L05**
  - Shutdown requests MUST resolve deterministically.
  - Resolution MUST result in `Finalized` or a documented failure outcome.
  - Nodes MUST NOT remain indefinitely in a transition state.
- **SPEC_L06**
  - Deactivation logic MUST run before shutdown logic when shutting down from `Active`.
  - This MUST manifest as deactivation effects/events before shutdown.
- **SPEC_L07**
  - While `Inactive`, lifecycle-managed publishers MUST NOT publish messages.
- **SPEC_L08**
  - Clients MUST NOT assume service unavailability based solely on lifecycle state.
- **SPEC_L09**
  - A compliant implementation MUST support lifecycle introspection via standard ROS 2 tools.

#### docs/spec/global/parameters.md
- **SPEC_P01**
  - Parameter semantics MUST be exposed via standard ROS 2 parameter services and messages.
- **SPEC_P02**
  - The chosen declaration mode MUST be documented.
  - If `allow_undeclared_parameters` is false, setting undeclared parameters MUST fail.
- **SPEC_P03**
  - Unknown/undeclared names MUST return `NOT_SET`.
- **SPEC_P04**
  - Parameter updates MUST be atomic per request.
  - Partial application MUST NOT be externally observable.
  - Rejected updates MUST NOT emit parameter change events.
- **SPEC_P05**
  - Parameter change events MUST describe state changes.
  - Observers MUST NOT rely on event count as a proxy for operation count.
  - Ordering and content MUST remain deterministic.
- **SPEC_P06**
  - `describe_parameters` on unknown parameters MUST be consistent.
  - If descriptors are returned, their semantics MUST be documented.
- **SPEC_P07**
  - Returned names MUST be complete (subject to filters).
  - Returned names MUST be stable across identical queries.
- **SPEC_P08**
  - If supported, deletion MUST be documented.
  - Deleted parameters MUST be reported via parameter events.

#### docs/spec/system/system_contract.md
- **SPEC_SYS01**
  - Implementations MUST make node ownership and lifetime explicit.
- **SPEC_SYS02**
  - Systems MUST support deterministic bringup and teardown sequences.
- **SPEC_SYS03**
  - Execution MUST occur only via executor/lifecycle/observable triggers.
- **SPEC_SYS04**
  - Failures MUST surface via lifecycle, action, or explicit errors.
- **SPEC_SYS05**
  - Systems MUST reflect true operational state in the ROS graph.
  - Graph state MUST be reliable for operators and tooling.
- **SPEC_SYS06**
  - Action servers MUST clean up goals from vanished/abandoned clients.
  - Cleanup MUST occur within a bounded observation window.

#### docs/spec/system/executor.md
- **SPEC_E01**
  - Systems MUST make executor ownership explicit.
  - Libraries MUST NOT start background spinning implicitly.
  - Background execution MUST be explicit, documented, and controllable.
- **SPEC_E02**
  - Systems MUST support bounded execution modes for deterministic testing.
  - `spin_once(timeout=0)` MUST return immediately if no work is available.
- **SPEC_E03**
  - Shutdown operations MUST be bounded and deterministic.
  - Shutdown MUST unblock active spin loops within a bounded time.
  - Shutdown MUST be idempotent.
  - Dropping executor/node handles MUST NOT cause deadlock.
  - The process MUST exit cleanly without repeated external interruption.
- **SPEC_E04**
  - Systems MUST define and document concurrency models.
- **SPEC_E05**
  - Callback failures MUST NOT silently corrupt execution.
  - Failure handling policy MUST be documented.
  - Terminating failures MUST be explicit and observable.
  - Long-running callbacks MUST NOT permanently block shutdown.
- **SPEC_E06**
  - Executor and lifecycle semantics MUST compose safely under load.
  - Lifecycle transitions MUST NOT require re-entrant spinning.
  - Transition handling MUST NOT deadlock under load.
  - Busy rejection semantics MUST remain deterministic.
- **SPEC_E07**
  - Executor behavior MUST be testable via the harness.

#### docs/spec/system/composition.md
- **SPEC_C01**
  - Containers MUST expose standard ROS 2 composition services.
- **SPEC_C02**
  - Loaded nodes MUST become observable on the graph within the Settling Window.
  - Loaded nodes MUST be reported by `list_nodes`.
  - Node entities MUST be active and observable.
  - Failure reason MUST be observable on rejection.
- **SPEC_C03**
  - Unloaded nodes MUST be removed from `list_nodes`.
  - Entities MUST be destroyed and removed from the graph.
  - Nodes MUST cease graph participation.
  - Containers MUST NOT mutate state on rejection.
- **SPEC_C04**
  - Each loaded node MUST receive a unique identifier.
  - Identifiers MUST remain stable from load to unload.
- **SPEC_C05**
  - Concurrent load/unload MUST be handled deterministically.
  - Requests MUST be serialized or rejected without mutating state.
  - Busy rejections MUST be documented and observable.
- **SPEC_C06**
  - Parameter overrides/remappings MUST apply immediately on instantiation.
  - Deviations MUST be documented.
- **SPEC_C07**
  - Containers MUST be compatible with `ros2 component` CLI tooling.

#### docs/spec/system/ecosystem.md
- **SPEC_ECO01**
  - Implementations MUST expose lifecycle behavior compatible with external managers.
  - Lifecycle state/transitions/failures MUST be observable and controllable via standard services.
- **SPEC_ECO02**
  - Active components MUST provide a liveness detection mechanism.
  - Crashed/unresponsive components MUST be detectable.
  - Orchestration MUST react deterministically to loss of liveness.
- **SPEC_ECO03**
  - Superseded goals MUST transition to a terminal state indicating supersession.
  - Supersession MUST be represented as `CANCELED` in the baseline profile.
  - Supersession MUST NOT be represented as `ABORTED`.
- **SPEC_ECO04**
  - Implementations MUST support ecosystem tooling expectations.
- **SPEC_ECO05**
  - Implementations MUST behave indistinguishably via standard ROS 2 tools.

---

### D2) Orphaned specs, scenarios, and mismatched IDs

#### Orphaned or mismatched spec references
- **Scenario semantics reference non-existent spec IDs** (`SPEC_C08`, `SPEC_C10`, `SPEC_C13`, `SPEC_C15`) that are not defined in `docs/spec/system/composition.md`.

#### Orphaned scenarios or missing harness implementations
- Most scenarios in `harness/scenarios/*` are empty skeletons (`ops: []`, `expects: []`), so the Spec → Scenario → Harness → Trace chain is currently non-functional.
- Several scenarios use fields not allowed by the scenario schema (`spec_ref`, `status`, `type`).

#### Mismatched ID patterns
- Scenario semantics for ecosystem uses **H00** in a spec mapping (`SPEC_ECO05`), but no semantics entry exists for H-series in the semantics files; H00 is a harness-only self-test.
- The coverage matrix includes appended rows with inconsistent formatting and concatenated table data, making it non-machine-readable.

---

### D3) Untestable clauses and timing claims lacking measurement definitions
- Multiple system contracts require “deterministic” or “bounded” behavior without specifying concrete observation windows in the spec itself.
- Claims like **“No hidden execution”** and **“Behavioral indistinguishability”** are not operationally testable without additional measurement scaffolding or observability hooks.
- Requirements that force “observable” behavior do not define minimum evidence classes required for a pass/fail verdict.

---

## E) Spec-vs-upstream comparison table (by domain)

**Note:** External links and ROS 2 documentation could not be verified in this environment (network blocked). The table below lists what the repo *claims* as upstream sources and flags verification status as **NOT VERIFIED**.

| Domain | Spec references | Upstream source(s) cited | Verification status | Notes |
| --- | --- | --- | --- | --- |
| Actions | SPEC_AC01..AC07, SPEC_A01..A09 | ROS 2 design article on Actions | NOT VERIFIED | Upstream citations exist in spec text; no local snapshots or quotes included. |
| Lifecycle | SPEC_LC01..LC05, SPEC_L01..L09 | ROS 2 design article on Node Lifecycle | NOT VERIFIED | Several lifecycle constraints appear stricter than upstream docs (requires confirmation). |
| Parameters | SPEC_PC01..PC08, SPEC_P01..P08 | ROS 2 Jazzy parameters docs | NOT VERIFIED | Some clauses (event cardinality, list stability) are likely derived from baseline behavior rather than docs. |
| Composition | SPEC_C01..C07 | ROS 2 composition design article | NOT VERIFIED | Several transactional semantics are BIC extensions beyond upstream docs. |
| Executor/System | SPEC_E01..E07, SPEC_SYS01..SYS06 | None (upstream silent) | NOT VERIFIED | These are BIC requirements; must be explicitly marked as baseline-derived and validated. |
| Ecosystem | SPEC_ECO01..ECO05 | None (upstream silent, Nav2 referenced) | NOT VERIFIED | Requires explicit provenance tagging and validation traces. |

---

## F) Harness readiness review (capabilities, evidence discipline, windows)

### Observability and evidence discipline
- The harness contract is strong in principle (backend observes; core judges), but the **trace schema, scenario schema, and scenario bundles are inconsistent**.
  - Scenario bundles expect event types (`goal_send`, `goal_response`, `terminal_set_attempt`) that are not present in the trace schema.
  - Scenario bundles use fields (`spec_ref`, `status`, `type`) not allowed by the scenario schema.
- Capability declarations are described in methodology but are not enforced by the scenario schema or scenario bundles.

### Observation and settling windows
- The measurement model is defined in `oracle_measurement.md`, but specs use “bounded time” without referencing which window applies.
- No harness settings or constants for settling windows are documented in code or configuration.

### Implementation status
- The majority of scenarios are stubs with empty `ops` and `expects` arrays.
- Only a few scenarios (e.g., A01, P06, P12, H00) include concrete operations/expectations.

---

## G) Repo hygiene / contributor UX review (CI, templates, structure, naming)

### Strengths
- Clear directory layout and separation of specs, provenance, harness, and reference implementation.
- CONTRIBUTING clearly explains process constraints and what is out of scope.

### Gaps
- No CI configuration present (no automated link checks, schema validation, or traceability checks).
- No issue/PR templates to enforce provenance tagging and scenario mapping.
- No automated consistency checks for spec→scenario ID mapping or schema validation.
- Scenario coverage matrix is manually maintained and currently malformed.

---

## H) Reception analysis (“political framing” + recommended edits)

### How this will be received
- **ROS 2 core maintainers**: Will appreciate the emphasis on testable semantics and traceability, but may object to language implying this repo defines “truth” where upstream is silent. They are likely to push back on BIC requirements framed as normative without direct upstream quotes or validated traces.
- **Ecosystem maintainers (Nav2, etc.)**: Will value explicit contracts but may reject constraints that appear to encode Nav2-specific policy as ROS-wide semantics (e.g., supersession policy).
- **Safety/industrial users**: Will want evidence artifacts and deterministic reproduction. The current lack of validated traces and tagged provenance will undermine certification posture.
- **General community**: Tone is assertive (“THE LAW”, “semantic truth”), which can trigger defensiveness. Careful phrasing is needed to position this as a measurement instrument rather than a competing standard.

### Recommended wording tweaks to reduce friction
- Replace “THE LAW” and “semantic truth” with “baseline contract set” or “measured baseline profile.”
- Explicitly mark BIC rules as **baseline-derived** and **revocable** in each spec clause, not just in the system contract.
- Add a short “Upstream deference” callout in each spec file: “These clauses are non-authoritative if upstream provides a conflicting normative statement.”

---

## I) Prioritized action list

### P0 — Breaks correctness/credibility
1. **Add provenance tags** (`VALIDATED/UNVALIDATED/REJECTED`) to every normative clause.
2. **Fix schema mismatches** between scenario bundles, scenario schema, and trace event schema.
3. **Correct spec/scenario ID mismatches** (remove or define SPEC_C08/C10/C13/C15 or update scenario semantics).
4. **Repair coverage matrix formatting** and make it machine-verifiable.
5. **Add CI checks** for schema validation and traceability consistency.

### P1 — Improves auditability
1. Add local extracts/quotes from upstream ROS docs (or cached snapshots) to each spec’s provenance section.
2. Add a script to extract and validate normative clauses + provenance tags.
3. Implement minimal harness scenarios for each domain (A/L/P/S/G) with deterministic expectations.
4. Track evidence artifacts (rosbag/trace) in a structured `trace_records/` folder as per provenance README.

### P2 — Nice-to-have
1. Add contributor templates for divergence records and missing semantics entries.
2. Publish a public “baseline profile manifest” (RMW, ROS distro versions, Nav2 versions, Docker digests).
3. Add “spec version” metadata to scenario bundles and trace outputs for historical comparison.

---

## Appendix: Local link hygiene check
- Local relative links resolved without missing file targets.
- External links not verified due to network restrictions.

