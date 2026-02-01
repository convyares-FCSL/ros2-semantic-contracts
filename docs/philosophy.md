# Philosophy — Semantic Contracts for ROS 2

This document defines how this repository establishes, constrains, and evolves **semantic truth** for ROS 2 core behaviour.

It is not explanatory, motivational, or pedagogical.
It records the rules under which specifications in this repository are written, enforced, and versioned.

---

<details open>
<summary><strong>1) What is the source of semantic truth?</strong></summary>

**Question**
Where does authoritative ROS 2 behaviour come from?

**Answer**
From upstream ROS documentation and REPs where they are complete; otherwise from the current production stack: **ROS 2 Jazzy + rclcpp + Nav2**.

**Understanding**
This repository does not invent behaviour.
It **codifies existing behaviour** into explicit, testable semantic contracts.

Upstream specifications are authoritative when they are precise.
Where they are incomplete, ambiguous, or silent, the observed behaviour of the production stack defines the baseline truth.

**Criteria for Derived Truth**
A behavior observed in the production stack becomes normative only if:
1.  It is **consistent** across repeated measurements in the baseline profile (rclcpp + Jazzy).
2.  At least one widely used downstream consumer (e.g., Nav2) **relies on it** for correctness.

Incidental artifacts (e.g., specific error strings or timing jitter) that do not affect consumer correctness remain implementation details, not semantic contracts.

</details>

---

<details>
<summary><strong>2) What is the role of this repository?</strong></summary>

**Question**
What does this repository do that upstream ROS does not?

**Answer**
It defines **normative semantic contracts** where upstream definitions stop at mechanisms or interfaces.

**Understanding**
ROS 2 provides interfaces, APIs, and mechanisms sufficient to build working systems.
It does not provide a single, canonical, testable definition of semantic correctness for lifecycle, actions, or parameters.

This repository fills that gap by making semantic behaviour:

* explicit,
* transport-agnostic,
* language-agnostic,
* and enforceable via tests.

It does not replace ROS documentation, quality processes, or tests.
It complements them by defining **what correctness means**.

</details>

---

<details>
<summary><strong>3) Are the specifications normative?</strong></summary>

**Question**
Are the specifications in this repository descriptive or authoritative?

**Answer**
They are **normative**.

**Understanding**
Specifications use normative language (“MUST”, “MUST NOT”, “SHALL”) because they define a contract.

They do not describe possibilities or recommendations.
They define required behaviour for any implementation claiming conformance to a given semantic profile.

Material that describes system-level "hygiene" or design intent without black-box testable invariants is classified as **guidance**, not specification.

</details>

---

<details>
<summary><strong>4) What is the relationship between specifications and tests?</strong></summary>

**Question**
When specifications and executable oracles disagree, which is authoritative?

**Answer**
The specification is authoritative. Tests enforce the specification.

**Understanding**
Tests do not define truth.
They detect divergence.

Any disagreement between spec and observed behaviour results in a **documented divergence**, with one of three outcomes:

* the specification is corrected,
* an upstream regression is identified,
* or an intentional upstream change is adopted via a deliberate, versioned update.

Silent drift is not permitted.

</details>

---

<details>
<summary><strong>5) How is non-determinism handled?</strong></summary>

**Question**
Can contracts declare behaviour as non-deterministic?

**Answer**
Only when outcomes cannot be bounded without introducing implementation assumptions.

**Understanding**
Contracts must constrain behaviour to a **deterministic envelope** wherever possible.

When ordering or timing cannot be fixed, contracts define:

* allowed outcome sets,
* invariants that must always hold,
* and forbidden states or transitions.

“Unspecified” is not an acceptable substitute for bounded semantics. Negative assertions (e.g., "X must not happen") must always be bounded by a defined **Observation Window**.

</details>

---

<details>
<summary><strong>6) Do contracts include failure behaviour?</strong></summary>

**Question**
Are failure responses part of the semantic contract?

**Answer**
Yes.

**Understanding**
Contracts specify not only valid behaviour, but also the required response to invalid input, invalid state, or illegal transitions.

Silence, rejection, error reporting, or no-op are all semantic choices and must be defined explicitly.

</details>

---

<details>
<summary><strong>7) What is the scope of “core” semantics?</strong></summary>

**Question**
What belongs in core semantic contracts?

**Answer**
Only semantic correctness, safety, and liveness invariants.

**Understanding**
Core contracts define *what must be true*, not *how efficiently it is achieved*.

Performance characteristics belong in benchmarks, harnesses, and provenance documentation unless they directly affect correctness or liveness. When timing is part of correctness, it is treated explicitly as a liveness constraint.

</details>

---

<details>
<summary><strong>8) How are ecosystems and reference stacks treated?</strong></summary>

**Question**
What role do mature ecosystems such as Nav2 play?

**Answer**
They act as **validation pressure** and evidence for undefined semantics.

**Understanding**
Where upstream ROS specifications are incomplete, behavior relied upon by the ecosystem defines the semantic baseline.

Nav2 is **evidence of dependency**, not the authority itself.
Behavior required by Nav2 is elevated to a normative ROS 2 contract because breaking it breaks the ecosystem, not because Nav2 defines ROS 2.

</details>

---

<details>
<summary><strong>9) How does this repository evolve over time?</strong></summary>

**Question**
How are changes across ROS distributions handled?

**Answer**
Through explicit, versioned semantic contracts.

**Understanding**
Each contract set is tied to a specific upstream baseline (ROS distribution, client library, and ecosystem stack) and is versioned explicitly.

Earlier contract sets are retained as historical baselines.
At any point in time, only one contract set is considered authoritative.

Future distributions are compared against existing contracts.
Differences are treated as regressions until proven intentional via upstream evidence.

Intentional changes result in a new contract version, not silent mutation.

</details>

---

<details>
<summary><strong>10) What does this repository explicitly refuse to be?</strong></summary>

**Question**
Are there hard exclusions?

**Answer**
Yes.

**Understanding**
This repository refuses to be:

* a tutorial,
* a style guide,
* a feature catalogue,
* a language advocacy project,
* or a replacement for ROS documentation.

Its sole purpose is to define and enforce semantic truth.

</details>