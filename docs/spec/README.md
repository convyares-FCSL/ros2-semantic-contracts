`# ROS 2 Semantic Specifications

This directory contains the normative contracts that define the "Professional Baseline" of ROS 2.
These specifications are organized into **three distinct abstraction layers**, separating "Physics" from "Protocol" from "Operation."

---

## The Three-Layer Architecture

### 1. Core Contracts (`core/`)
**"The Physics Engine"**
* **What it is:** Transport-agnostic state machine rules.
* **Enforcement:** Enforced by the core logic engine (internal library state).
* **Example:** "A Goal ID must not be reused." "Lifecycle transitions must be sequential."
* **Files:**
    * [`action_core.md`](core/action_core.md)
    * [`lifecycle_core.md`](core/lifecycle_core.md)
    * [`parameter_core.md`](core/parameter_core.md)

### 2. Global Specifications (`global/`)
**"The Wire Protocol"**
* **What it is:** Observable behavior over DDS/ROS 2 services and topics.
* **Enforcement:** Enforced by the RMW/RCL layer and observers.
* **Example:** "Transitions emit `TransitionEvent` messages." "Action servers expose `/_action/send_goal`."
* **Files:**
    * [`action.md`](global/action.md)
    * [`lifecycle.md`](global/lifecycle.md)
    * [`parameters.md`](global/parameters.md)

### 3. System Contracts (`system/`)
**"The Runtime Reality"**
* **What it is:** Operational constraints required for a stable production system.
* **Enforcement:** Enforced by the Executor, Composition container, and OS process model.
* **Example:** "Shutdown must be clean." "Nodes must be unique in a container." "No hidden background threads."
* **Files:**
    * [`system_contract.md`](system/system_contract.md) (The root contract & BIC definition)
    * [`composition.md`](system/composition.md)
    * [`executor.md`](system/executor.md)
    * [`ecosystem.md`](system/ecosystem.md)

---

## Normativity & Validation

* **Normative Requirements:** Marked with `MUST` / `SHALL`. These are legally binding for compliance.
* **Baseline Interoperability Constraints (BIC):** Requirements derived from ecosystem necessity (e.g., Nav2 operability) where upstream specs are silent. Defined in [`system_contract.md`](system/system_contract.md).
* **Unvalidated Hypotheses:** Marked `⚠️ UNVALIDATED`. These describe expected behavior that has not yet been proven by the Oracle.

---

## Spec ↔ Scenario linking conventions

**Scenario `Layer:` field** — In `docs/provenance/scenario/semantics_*.md` the `Layer:` value (`Core` / `Global` / `System` / `Policy` / `Tooling`) records the *functional level exercised by the scenario*, not the file-path layer of the spec it validates. A scenario labelled `Core` may validate a spec defined in `global/` when the exercised property is a core invariant projected onto the wire.

**Cross-layer spec duplicates** — Some obligations appear at two layers (e.g. `SPEC_SYS06` in `system_contract.md` and `SPEC_A09` in `global/action.md`). Convention: the System-layer spec is the *originating* obligation (typically a BIC); the Global-layer spec is a *projection* — a restatement scoped to the wire-observable interface. Scenarios validate against the originating obligation; the projection inherits coverage implicitly. The originating spec carries the scenario tag.

**Deferred / infra-delegated specs** — Specs whose core assertion requires adversarial stimulus or manual analysis not expressible in the current ops model are labelled `Automation note — deferred`, `— manual/deferred`, or `— infra-delegated` inside their `<details>` block. Tagged scenarios remain as *design intent* but do not constitute exercised coverage until the required primitive is available. Matching `⚠️ Status:` labels appear on the scenario side in `semantics_*.md`.

See `docs/provenance/` for the validation methodology.