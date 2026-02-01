# Oracle Measurement Definitions

This document defines the **epistemology of the oracle**: the rules for observation, measurement, and judgment.
These definitions are normative for all `docs/spec` requirements that rely on timing or observability.

---

## 1. Observation Window ($T_{obs}$)

**Definition:**
A bounded duration during which the Oracle waits for an expected event to occur.

**Semantic mapping:**
* **Liveness:** "Node X MUST become ready" $\rightarrow$ "Node X MUST become ready within $T_{obs}$."
* **Negative Proofs:** "Event Y MUST NOT happen" $\rightarrow$ "Event Y MUST NOT happen within $T_{obs}$."

**Verdicts:**
* **PASS:** Expectation met within $T_{obs}$.
* **FAIL:** Explicit invariant violation observed (e.g. wrong data).
* **INCONCLUSIVE:** $T_{obs}$ expired without the expected event (Timeout).

---

## 2. Settling Window

**Definition:**
A specific $T_{obs}$ used after a stimulus (e.g. `load_node`) to allow for:
* RMW discovery and graph propagation
* Executor scheduling latency
* Network transport settling

**Standard Value:**
Implementation-defined by the Harness (typically **5-10s** for DDS/RMW discovery).
All "Immediate" requirements in the specs are implicitly bounded by the Settling Window.

---

## 3. Verdict Definitions

The oracle distinguishes between **Logical Failures** and **Observational Limits**.

### FAIL (Hard Regression)
The system actively violated a constraint.
* *Example:* State transition to `X` when only `Y` is valid.
* *Example:* UUID reuse detected.

### INCONCLUSIVE (Soft Failure)
The system failed to produce evidence within the Observation Window.
* *Example:* `load_node` call did not return within 10s.
* *Implication:* Could be performance regression, deadlock, or network issue. Not a semantic contradiction, but a functional failure.

---

## 4. Evidence Class

**Definition:**
A category of observable trace data required to judge a scenario.

* **Lifecycle Events:** `/transition_event` topic data.
* **Action Feedback:** Goal status arrays and feedback messages.
* **Parameter Events:** `/parameter_events` topic data.
* **Graph State:** `ros2 node list` / `ros2 topic list` snapshots.

If a backend cannot produce a specific Evidence Class, scenarios requiring it are **SKIPPED**.