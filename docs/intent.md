# Intent — ros2-semantic-contracts

This repository exists to define, version, and enforce **normative semantic contracts** for ROS 2 core behaviour.

It establishes a **versioned, testable semantic profile** so that ROS 2 behaviour stops being
“whatever the reference implementation happens to do” without leaving a paper trail.

We achieve this by maintaining:
1. **Semantic Contracts:** Transport-agnostic definitions of Core, Global, and System invariants.
2. **The Oracle:** A semantic harness that validates any backend against these contracts.
3. **Divergence Records:** A ledger of exactly *where* and *why* an implementation differs.

---

## What This Repository Is

This repository is:
* A **semantic definition layer grounded in upstream documentation and production evidence**.
* A baseline definition derived from a named profile (**ROS 2 Jazzy + rclcpp**).
* A mechanism to make **semantic drift visible, reviewable, and versioned**.

It defines *what must happen* — not *how to implement it*.

---

## What This Repository Is Not

* **Not a Tutorial:** It does not teach ROS; it defines semantic truth.
* **Not a Best-Practices Guide:** It defines strict boundaries, not advice.
* **Not a Performance Benchmark:** It measures correctness, not speed
  (except where latency violates a contract).

---

## Relationship to Client Libraries

This repository does not define APIs.
It defines **required behaviour**.

Where upstream ROS specifications are incomplete or ambiguous, the behaviour of the
**Reference Stack (ROS 2 Jazzy + rclcpp)** is treated as the semantic anchor.

Other client libraries (including extended variants such as
**`rclrs + rosrustext_rclrs`**) are evaluated relative to this anchor.

### The Role of Ecosystem Consumers (Nav2)

Complex ecosystem stacks like **Nav2** are used as **evidence amplifiers**.

Nav2 is **not** treated as an authority on semantics.
Instead, it is used to *surface hidden system invariants* that the ecosystem implicitly relies upon.
If breaking a specific behaviour breaks Nav2, that behaviour is a strong candidate for a
previously undocumented system contract — which must then be validated directly against
the production stack.

---

## Validation Roadmap

Our strategy is **comparative verification**.
We do not force all libraries to be identical; we force them to be **explicit about their differences**.

### Phase 1: The Golden Master (Current)

* **Target:** `rclcpp` (C++) on **ROS 2 Jazzy**
* **Role:** Semantic anchor when upstream documentation is ambiguous
* **Goal:** Prove that the specifications describe the actual behaviour of the reference stack

### Phase 2: The Challengers (Verification)

We run the Oracle against these targets to detect semantic drift:

* **`rclpy`** — Python client (flexible, widely used baseline)
* **`rclrs`** — Rust client (safety-oriented baseline)
* **`rclgo`** — Go client (systems / microservice-oriented baseline)
* **`rclrs + rosrustext_rclrs`** — Rust parity extension layer
  * Closes feature gaps relative to `rclcpp`
  * Adds lifecycle-aware subscriptions, services, timers
  * Introduces ergonomic Rust improvements (e.g. builders, safer abstractions)

These targets are not required to converge on identical behaviour.
Any divergence must be **observable, documented, and justified**.

### Phase 3: Distribution Regression

* **ROS 2 Humble** — Quantify behavioural differences across LTS versions
* **ROS 2 Rolling** — Detect breaking semantic changes *before* release

---

## Scope Discipline

Only behaviour that affects **semantic correctness, safety, or liveness**
belongs in this repository.

Performance characteristics, tuning guidance, and deployment advice are
**out of scope** unless they directly affect correctness
(e.g. deadlock prevention).

This constraint keeps the contract set **minimal, enforceable, and stable**.

---

## Success Metric

Success is **not** “All backends pass 100%.”

Success is:

> **“An engineer can look at the Divergence Record and make an informed architectural decision.”**

Examples:
> “I need exact parameter atomicity, so I cannot use Library X yet.”  
> “I need strict lifecycle ordering, so I must use Library Y.”
