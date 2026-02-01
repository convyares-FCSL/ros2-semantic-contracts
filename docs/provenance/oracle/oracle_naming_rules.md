# Oracle Naming & Traceability Rules

This document defines the naming conventions that link **Specification Invariants** to **Oracle Scenarios** to **Audit Verdicts**.

The goal is **one-to-one traceability** from the text of the spec to the JSON trace of the execution.

---

## 1. Spec ID Scheme (`<DOMAIN><NUM>`)

Stable identifier for one atomic semantic invariant.

* **A** = Action semantics (Core)
* **L** = Lifecycle semantics (Core)
* **P** = Parameter semantics (Core)
* **C** = Composition semantics (System)
* **E** = Executor semantics (System)
* **S** = General System/Ecosystem semantics (System)
* **G** = Global/Generic semantics

**Examples:** `A01`, `L10`, `S15`.

**Rules:**
* IDs must be stable. Do not renumber.
* New invariants get the next available number.

---

## 2. Scenario ID Scheme (`<Spec ID>_<Summary>`)

Stable identifier for one runnable oracle test case.

**Format:** `<SpecID>_<short_snake_case_summary>`

**Examples:**
* `A01_unique_goal_identity`
* `L05_concurrent_transition_rejected`
* `S19_graph_hygiene`

**Rules:**
* **Cardinality:** Default is **1 Spec : 1 Scenario**.
* **Splitting:** If a Spec requires multiple tests, use suffixes: `A01_identity_creation`, `A01_identity_rejection`.
* **Stability:** These IDs appear in JSON logs. Changing them breaks historical analytics.

---

## 3. Layer Coverage Rules

The `Layer` defined in the spec determines the Oracle strategy:

* **Core:** Enforced by the `oracle_a` (stub/mechanistic) runner.
* **System/Global:** Enforced by the `oracle_b` (integration) runner (e.g., against Dockerized Nav2).
* **Tooling:** Validated by `oracle_b` via CLI invocation.

---

## 4. Trace Schema Requirements

Every scenario execution MUST emit the following JSON events (minimum vocabulary):

1.  **`scenario_start`**: `{ "id": "A01_...", "backend": "ros" }`
2.  **`assertion`**: `{ "ok": true, "expected": "...", "observed": "..." }`
3.  **`scenario_end`**: `{ "ok": true, "verdict": "PASS" }`

The Audit Tool computes the final verdict solely from these events.