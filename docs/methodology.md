# Methodology — Contract Production and Validation

This document defines the **mechanical process** by which semantic contracts in this repository are authored, validated, enforced, and evolved.

It does not define authority, scope, or baseline selection.
Those are fixed elsewhere.

---

## Scope

This methodology applies to all normative material in this repository, including:

- core semantic contracts (`*_core.md`)
- global semantic specifications
- system and ecosystem contracts
- oracle and harness validation artifacts
- divergence records and traceability matrices

It does not apply to adapter implementations or parity ledgers.

---

## Contract Authoring

All contracts are written as **normative specifications**.

Rules:

- Normative language (“MUST”, “MUST NOT”, “SHALL”) is used only for enforceable requirements.
- Each requirement must be:
  - observable,
  - testable, or
  - explicitly classified as a hypothesis.
- Contracts define behaviour, not implementation strategy.

Contracts are authored in one of the following categories:

- **Core Contracts** — engine-enforceable semantic invariants
- **Global Specifications** — ROS-facing observable semantics
- **System Contracts** — operability and orchestration constraints
- **Ecosystem Contracts** — interoperability expectations

---

## Provenance and Traceability

Every normative requirement MUST be traceable to at least one of:

- upstream ROS documentation or REPs
- upstream implementation behaviour
- ecosystem-defined practice
- explicit project policy

Traceability is recorded via:

- provenance sections in specification documents
- citation tables
- traceability matrices mapping:
  - contract clause → upstream reference → harness validation

Untraceable requirements are not permitted.

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

A contract requirement is considered validated only when:

- a corresponding oracle or harness test exists, and
- the test passes against the baseline profile.

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
- Engine-enforceable invariants vs system-level constraints
- Specification vs validation

Any material that blurs these boundaries is non-compliant.

---

## Exit Conditions

A contract or section may be considered complete only when:

- all requirements are traceable,
- all hypotheses are resolved or explicitly retained,
- validation status is clear,
- and no ambiguity remains regarding scope or authority.
