# Boundary Contract — Actor / Backend Separation & Scenario Provenance

**Authority:** Normative. Single source of truth for component roles and provenance rules.
**Scope:** All scenario families. Machine-enforced scope: Actions (Axx) initially.
**Referenced from:** `docs/CONTEXT.md`, `CLAUDE.md`

---

## 1. Components & Roles

### Backend

The **backend** is a runner, observer, and tracer.

It is responsible for:

* instantiating ROS entities (nodes, clients, servers),
* sending stimuli via standard ROS interfaces,
* observing externally visible ROS behaviour,
* emitting trace events.

The backend MUST NOT encode semantic outcomes.
It orchestrates *what happens*, never *what should happen*.

---

### Actor

An **actor** is a reusable ROS component (node or set of nodes).

Actors:

* use real ROS APIs,
* expose behaviour only through standard ROS mechanisms,
* are reusable across scenarios,
* are **scenario-agnostic**.

Actors MUST NOT:

* read scenario JSON or bundles,
* read scenario-derived files, sockets, or side channels,
* change behaviour based on scenario identity or expectations.

Actors may have **behavior profiles** only if those profiles:

* are part of a stable, documented actor API,
* are reused across multiple scenarios,
* are selectable only via public ROS mechanisms (see §2.3).

---

### Scenario

A **scenario** defines:

* ordering of operations,
* expected observations over time.

Scenarios:

* MAY choose inputs and stimuli,
* MUST NOT choose outcomes,
* NEVER steer accept/reject/terminal behaviour via hidden control paths.

Scenarios author **expectations**, not **answers**.

---

## 2. The Hard Rule (Provenance Rule)

> **Evidence is only meaningful if outcomes are not determined by scenario payload or hidden control channels.**

A component constitutes a **self-confirming fixture** if its behaviour is shaped to satisfy a scenario’s expected outcome rather than constrained by the protocol under test.

This makes the scenario unfalsifiable.

A fixture is **self-confirming** when **all** of the following hold:

1. Its behaviour is influenced by scenario payload, bundle content, or scenario-derived control channels.
2. It produces the outcome that the scenario’s `expects` block asserts.
3. That outcome could not reasonably differ under the same protocol interaction.

**Important:**
Using real ROS APIs (e.g. `rclcpp_action`) does NOT exempt a component.
What matters is **decision autonomy**, not API fidelity.

This closes the gap left by `RELIABILITY_RULES.md`, which governs realism of calls but not provenance of decisions.

---

## 3. Allowed vs Forbidden Configuration

### Allowed (Public, Observable)

Actors may be configured via:

* ROS parameters (at startup),
* standard ROS interactions (topics, services, actions),
* environment variables **only** for runtime plumbing (namespaces, logging, ports).

These mechanisms must be:

* documented,
* stable,
* usable by ordinary ROS users.

---

### Forbidden (Scenario-Shaped Control)

Actors MUST NOT:

* read scenario JSON or bundles,
* read files/sockets written by the scenario or backend for control,
* expose custom “control topics/services” whose sole purpose is to steer outcomes (e.g. `/set_mode reject_for_A01`),
* implement behaviour profiles that exist only to satisfy a single scenario.

A ROS interface that exists only to make one scenario pass is still a fixture.

---

## 4. Actions (Axx) — Boundary Clarification

Action scenarios validate **ROS action protocol semantics as observed on global streams**.

Key clarification:

* The **stimulus** is client-originated (goal submission).
* The **observable evidence** (status, feedback, result) is published by the server.
* The contract is about the **protocol**, not about privileging client or server as “the truth”.

Therefore:

* Axx scenarios may choose either side as the SUT depending on how the stimulus is applied (client library vs CLI).
* What matters is that **no component authors the outcome on behalf of the scenario**.

For Actions (Axx), the following enforcement applies initially:

1. Actors involved in accept/reject/terminal decisions MUST be scenario-agnostic.
2. No accept/reject/terminal outcome may be selected via scenario payload or private control paths.
3. Evidence MUST arise from real ROS action interactions and be observable on standard streams.

Whether an action server runs inside the backend or externally is secondary; **scenario-shaped behaviour is the disallowed axis**.

---

## 5. Other Families

Lifecycle (Lxx), Parameters (Pxx), and other families follow the same provenance rule:

* If the spec concerns **server semantics**, the backend may host the server as the SUT.
* If the spec concerns **client semantics**, the backend hosts the client and interacts with a generic actor.

In all cases:

* the component under test may not control its own verdict,
* peers must not be scripted to satisfy expectations.

Family-specific refinements will be added in future revisions.

---

## 6. Relationship to Existing Constraints

| Constraint                              | Governs                       | Does NOT govern         |
| --------------------------------------- | ----------------------------- | ----------------------- |
| `RELIABILITY_RULES.md` §Strict Fidelity | Real ROS API usage            | Outcome provenance      |
| `CLAUDE.md` §Non-negotiables            | No simulated behaviour        | Scenario-shaped control |
| **This document**                       | Outcome autonomy & provenance | —                       |

---

## Summary (Non-Negotiable)

* The enemy is **scenario-shaped behaviour**, not “peers”.
* Code location (backend vs external) is secondary to **control provenance**.
* Actors must be reusable, scenario-agnostic, and protocol-constrained.
* Scenarios define **when** and **what to observe**, never **what must happen**.

This boundary is what keeps evidence meaningful.
