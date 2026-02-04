# Executor — System Contract

**Normativity Class: Baseline System Contract**

These requirements define interoperability constraints for the ROS 2 professional baseline.
They are validated via system-level oracle and harness tests, not core semantic engines.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- executor ownership and spinning
- bounded shutdown and cancellation
- callback concurrency guarantees
- lifecycle interaction under load
- testability of execution behaviour
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- client-library APIs
- callback scheduling algorithms
- real-time guarantees
- QoS tuning
- component composition mechanics
</details>

---

## Executor ownership

### SPEC_E01 — Explicit Ownership and Control [S07]

The system MUST make executor ownership explicit to prevent race conditions and "hidden" background work.
- No callbacks may be **invoked** unless an executor is actively being spun.
- Libraries MUST NOT start background spinning implicitly.
- If background execution exists, it MUST be explicitly constructed, documented, and controllable.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Hidden spinners forbidden)

</details>

---

## Spin modes

### SPEC_E02 — Bounded Spin Modes [S08]

The system MUST support bounded execution modes sufficient for deterministic integration and testing.
- **Spin Forever:** Block until shutdown signal.
- **Spin Once / Step:** Process a bounded unit of work.
- **Spin with Timeout:** Execute for a bounded duration.
- **Non-Blocking Check:** `spin_once` with `timeout=0` MUST return immediately if no work is available.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on precise timing semantics)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Testability requirement)

**Notes**
- Exact API shape is implementation-defined, but the *semantics* of these modes must be observable.

</details>

---

## Shutdown semantics

### SPEC_E03 — Bounded and Idempotent Shutdown [S09]

Shutdown operations MUST be bounded and deterministic.
- A shutdown request MUST unblock any active spin loop within a bounded time (Settling Window).
- Shutdown MUST be idempotent (safe to call multiple times).
- Dropping executor or node handles MUST NOT cause deadlock.
- The process MUST exit cleanly without repeated external interruption.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on unblocking guarantees)
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Clean exit requirement)

</details>

---

## Callback concurrency model

### SPEC_E04 — Concurrency Model Documentation

The system MUST define and document its concurrency model.
- Whether callbacks may execute concurrently.
- Whether per-entity ordering is preserved.
- Whether starvation is possible.

> Derived / not independently validated: This is a documentation requirement; no single scenario can assert documentation adequacy. Exercised implicitly by any scenario that depends on concurrency behaviour.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Transparency requirement)

**Notes**
- This contract does **not** mandate a particular concurrency model (single vs multi-threaded), only that it is explicitly defined.
- Baseline expectation: Multi-threaded execution permits concurrency; ordering is best-effort.

</details>

---

## Callback isolation and failure handling

### SPEC_E05 — Failure Isolation and Observability [S10]

Callback failures MUST NOT silently corrupt execution.
- Failure handling policy MUST be documented.
- If a failure terminates execution, it MUST be explicit and observable (e.g. via process exit, error logging, or lifecycle transition).
- Long-running callbacks MUST NOT permanently block shutdown.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Safety requirement)

</details>

---

## Lifecycle interaction

### SPEC_E06 — Safe Lifecycle Composition [S11]

Executor and lifecycle semantics MUST compose safely under load.
- Lifecycle transitions MUST NOT require re-entrant spinning of the same executor.
- Transition handling MUST NOT deadlock under executor load.
- Busy rejection semantics MUST remain deterministic even under contention.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: Nav2 Ecosystem (lifecycle heavy usage)
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Deadlock prevention)

</details>

---

## Testing support

### SPEC_E07 — Testable Execution Interface

Executor behaviour MUST be testable via harness.
- Must support bounded spin/step capability.
- Must support deterministic shutdown.
- Must support reproducible lifecycle + executor interactions.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- REP/RFC: None
- Community: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Harness requirement)

**Notes**
- Tooling and test validation is performed via oracle and harnesses, not asserted by this document alone.
- Derived / not independently validated: No single scenario exercises "harness testability" as an isolated property. Coverage is implicit in every scenario that runs successfully through the harness.

</details>