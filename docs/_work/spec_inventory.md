# Spec Inventory and Boundary Classification (Workbench)

This document is a **temporary migration and boundary-classification workbench**.

Its purpose is to:
- classify existing specification material at a **section level**
- determine what belongs in Repo A (semantic authority) vs Repo B (adapter implementations)
- ensure that only **normative semantic material** enters the published spec set

This file is **not provenance** and **not normative**.
It may be deleted or emptied once migration is complete.

---

## Repository Roles (Reference)

**Repo A — Semantic Authority**
- Global semantic specifications
- Core semantic contracts
- System / operational contracts defining baseline behaviour
- Provenance (citations, baselines, divergence records)
- Oracle and harness tooling

**Repo B — Implementations**
- Adapter implementations
- Adapter-facing specs
- Parity ledgers mapping adapters to Repo A contracts

---

## Classification Labels

- **GLOBAL SPEC**  
  ROS-facing, observable semantic rules (services, topics, states, outcomes).  
  Language- and transport-agnostic.

- **CORE CONTRACT**  
  Semantics-engine-enforceable invariants only.  
  Deterministic envelopes; no tooling, no APIs, no policy.

- **SYSTEM CONTRACT**  
  Operational baseline semantics (executor, composition, ecosystem expectations).  
  Not enforceable by a core engine alone, but still normative at system level.

- **HARNESS EXPECTATION**  
  Behaviour validated via oracle/harness against the baseline stack  
  (e.g. CLI behaviour, tooling compatibility, Nav2 orchestration).

- **ADAPTER SPEC**  
  Language- or transport-specific obligations.

- **PARITY DOC**  
  Adapter parity ledger mapping an implementation to Repo A specs.

- **METHODOLOGY**  
  Process, philosophy, or explanatory material.

---

## Inventory (Section-Level)

> Note: Sections listed here are representative; actual migration MUST inspect content, not filenames.

### Lifecycle

| Source | Section / Content | Classification | Destination | Notes |
|---|---|---|---|---|
| `lifecycle.md` | State model, transitions, lifecycle services | GLOBAL SPEC | Repo A `docs/spec/lifecycle.md` | Normative ROS-facing semantics |
| `lifecycle.md` | CLI examples / tooling references | HARNESS EXPECTATION | Repo A `harness/` | Convert to oracle tests |
| `lifecycle_core.md` | Transition validity, busy rejection, error routing | CORE CONTRACT | Repo A `docs/spec/core/lifecycle_core.md` | Invariants only |

---

### Actions

| Source | Section / Content | Classification | Destination | Notes |
|---|---|---|---|---|
| `action.md` | Goal/Cancel/Result semantics, status meaning | GLOBAL SPEC | Repo A `docs/spec/action.md` | Observable semantics |
| `action.md` | Preemption behaviour inferred from Nav2 | SYSTEM CONTRACT | Repo A `docs/spec/action.md` (clearly marked) | Must cite Nav2 baseline |
| `action_core.md` | Goal lifetime, ordering, invariants | CORE CONTRACT | Repo A `docs/spec/core/action_core.md` | Engine-enforceable only |

---

### Parameters

| Source | Section / Content | Classification | Destination | Notes |
|---|---|---|---|---|
| `parameters.md` | Parameter identity, typing, events | GLOBAL SPEC | Repo A `docs/spec/parameters.md` | Normative |
| `parameters.md` | ros2 CLI behaviour | HARNESS EXPECTATION | Repo A `harness/` | Validate, don’t assert |
| `parameter_core.md` | Atomicity, ordering, rejection invariants | CORE CONTRACT | Repo A `docs/spec/core/parameter_core.md` | Engine-level semantics |

---

### System / Operational Specs

| Source | Section / Content | Classification | Destination | Notes |
|---|---|---|---|---|
| `executor.md` | Callback ordering, execution assumptions | SYSTEM CONTRACT | Repo A `docs/spec/executor.md` | Baseline semantics, not core |
| `composition.md` | Node composition, orchestration rules | SYSTEM CONTRACT | Repo A `docs/spec/composition.md` | Keep language-neutral |
| `core.md` | Professional stack expectations | SYSTEM CONTRACT | Repo A `docs/spec/system_core.md` | Rename to avoid collision |
| `ecosystem.md` | Baseline ecosystem behaviour | SYSTEM CONTRACT | Repo A `docs/spec/ecosystem.md` | MUST be language-neutral |
| `ecosystem.md` | Rust-specific requirements | ADAPTER SPEC | Repo B parity docs | Remove from Repo A |

---

### Other Material

| Source | Content | Classification | Destination | Notes |
|---|---|---|---|---|
| `methodology.md` | Spec-first methodology | METHODOLOGY | Repo A `docs/` | Non-normative |
| `professional_stack.md` | Context / explanation | METHODOLOGY | Repo A `docs/` | Informative only |
| Adapter parity docs | Adapter ↔ spec mapping | PARITY DOC | Repo B | One per adapter |

---

## Boundary Rules (Normative for Migration)

1. **CORE CONTRACT** files (`*_core.md`) contain *only* semantics-engine-enforceable invariants.
2. **GLOBAL SPEC** defines observable ROS-facing semantics, not tooling instructions.
3. Tooling and CLI compatibility are validated via **harnesses**, not asserted in specs.
4. Repo A specs are language-neutral; adapter- or language-specific content moves to Repo B.
5. System contracts are allowed in Repo A but must not masquerade as core contracts.

---

## Exit Criteria

This workbench file should be:
- empty, or
- removed

once all content is classified, relocated, and rewritten according to the above rules.
