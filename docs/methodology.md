# Methodology — Contract Production and Validation

This document defines the **mechanical process** by which semantic contracts in this repository are authored, validated, enforced, and evolved.

It does not define authority, scope, or baseline selection.
Those are fixed elsewhere.

---

## Scope

This methodology applies to all normative material in this repository, including:

- core semantic contracts (`*_core.md`)
- global semantic specifications
- oracle and harness validation artifacts
- divergence records and traceability matrices

Material that describes system hygiene or design intent without black-box testable invariants is classified as **guidelines**, not normative specifications.

---

## Contract Authoring

All contracts are written as **normative specifications**.

Rules:

- Normative language (“MUST”, “MUST NOT”, “SHALL”) is used only for enforceable requirements.
- Each requirement must be:
  - observable,
  - testable via black-box oracle, or
  - explicitly classified as a hypothesis.
- Contracts define behaviour, not implementation strategy.

Contracts are authored in one of the following categories:

- **Core Contracts** — engine-enforceable semantic invariants
- **Global Specifications** — ROS-facing observable semantics
- **Baseline System Contracts** — interoperability constraints validated via harness

---

## Provenance and Traceability

Every normative requirement MUST be traceable to at least one of:

- upstream ROS documentation or REPs
- upstream implementation behaviour (see "Derived Semantic Truth" below)
- explicit project policy

### Provenance Tagging

Provenance tracking is required for strict normative claims only. Any statement using **MUST**, **SHALL**, or **FORBIDDEN** must be tagged with one of the following in the specification source:

* `VALIDATED: <scenario_ids>` — Confirmed by passing oracle tests.
* `UNVALIDATED: <reason/hypothesis>` — Awaiting validation; currently a hypothesis.
* `REJECTED: <evidence_ref>` — Disproven by oracle evidence.

Descriptive text, `SHOULD`, and `MAY` statements do not require tags.

---

## Derived Semantic Truth (The Authority Ladder)

The repository’s specs are hypotheses about ROS 2 semantics. When a spec is ambiguous or contradicted by observation, the **production baseline** (ROS 2 Jazzy + rclcpp, under a declared RMW) is treated as the secondary authority only after behavior is reproduced and captured by oracle traces.

**Nav2** is used as a stress-test consumer that exposes ecosystem invariants, but Nav2 itself is not the authority: invariants inferred from Nav2 become normative only when confirmed by direct measurement against the production baseline.

---

## Capability Declaration

The harness acknowledges that not all backends support all semantic surfaces (e.g., a backend may support Pub/Sub but not Actions).

Rules:
- Backends MUST explicitly declare their supported **evidence classes** (capabilities) via trace events at runtime.
- The Core uses this declaration to determine test applicability.
- **SKIP**: The backend declared it does not support this capability.
- **FAIL**: The backend declared support but failed the semantic check.

Backend exit codes are transport-level signals only and do not determine semantic validity.

---

## Hypothesis Handling

Contracts MAY include sections marked:

> **⚠️ UNVALIDATED (baseline hypothesis)**

Rules for hypotheses:

- Hypotheses describe expected baseline behaviour not yet enforced by oracle tests.
- Hypotheses are scoped to the section they annotate.
- Hypotheses are non-normative until validated.
- Hypotheses MUST be promoted or removed before a contract version is finalized.

Hypotheses exist to make uncertainty explicit, not to weaken the contract.

---

## Validation and Enforcement

Validation is performed exclusively through:

- oracle execution against the defined baseline
- harness-driven interaction via standard ROS 2 interfaces
- observation of externally visible behaviour

Tests enforce contracts.
Tests do not define contracts.

### Observation Windows (Measurement Scaffolding)

Assertions that an event *does not happen* (negative proofs) or *has not yet happened* (liveness) MUST be bounded by a defined **Observation Window** (Settling Time).

The Observation Window is a **measurement tool**, not a system performance guarantee.

- **PASS**: The expected event occurred within the window.
- **FAIL**: The system actively violated a contract (e.g., invalid transition, wrong data) within the window.
- **INCONCLUSIVE**: The window expired without the expected event, and the spec does not mandate a specific timing guarantee.

A spec may only mandate a `FAIL` on timeout if the timing constraint is normative (e.g., `spin_once` with a generic timeout). Otherwise, timeouts represent measurement uncertainty, not necessarily semantic incorrectness.

---

## Divergence Management

Any mismatch between specified behaviour and observed baseline behaviour results in a **divergence record**.

Each divergence is classified as exactly one of:

- specification defect
- upstream regression
- intentional upstream change

Resolution requires an explicit action.
Silent alignment is forbidden.

---

## Versioning and Evolution

Contracts evolve only through explicit versioning.

Rules:

- Each contract set is tied to a named baseline profile.
- Changes are introduced as new versions, not edits in place.
- Historical versions are retained.
- At any point in time, only one contract set is authoritative.

Version changes require updated provenance and validation status.

---

## Separation of Concerns

This methodology enforces the following separations:

- Authority vs process
- Semantics vs implementation
- Engine-enforceable invariants vs system-level guidelines
- Specification vs validation

Any material that blurs these boundaries is non-compliant.