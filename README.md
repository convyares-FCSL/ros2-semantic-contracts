# ROS 2 Semantic Contracts

> **From "Implementation-Defined" to "Spec-Defined"**

ROS 2 has robust interface specifications, proven quality processes, and a mature testing culture.
What it does not yet have is a canonical, testable definition of **semantic correctness** for core behaviors—lifecycle orchestration, action goal handling, parameter lifecycle, executor semantics—independent of implementation.

In practice, semantic truth has emerged organically from the reference implementation (**rclcpp**) and mature production stacks (**Nav2**). This is natural and healthy. But as ROS 2 scales into multi-language ecosystems, safety-critical domains, and cross-middleware deployments, implicit semantics become harder to verify at scale.

This repository makes that implicit truth **explicit, normative, and testable**.

---

## The Problem

ROS 2 tells you *how to wire things together*.
It does not always specify *what correct behavior means* when systems are orchestrated, restarted, cancelled, preempted, or shut down under operational pressure.

This is not an oversight. Semantic convergence through implementation is normal in evolving ecosystems.
But it creates measurable costs:

* **Safety certification requires explicit contracts** — Implicit behavior cannot be formally verified.
* **Cross-language parity is assumed, not proven** — Subtle behavioral differences between C++, Python, and Rust remain undetected.
* **Regressions are hard to classify** — Is a change in Rolling a bug, a feature, or a misunderstanding?
* **"Works in practice" becomes indistinguishable from "works by accident"** — Critical invariants are inferred from reading source code, not specifications.

**This repository does not fix ROS 2.**
**It measures it.**

---

## The Thesis

**Semantic contracts should be explicit, normative, and testable.**

We define semantic contracts for ROS 2 core behavior: clear statements of required behavior, allowed outcomes, and failure responses, independent of language or transport.

These contracts are:
* **Normative** — they define what correct behavior is.
* **Language-agnostic** — they apply equally to C++, Python, Rust, or Go.
* **Transport-agnostic** — they do not depend on DDS implementation details.
* **Observable** — they are judged by externally visible behavior.
* **Enforceable** — they are backed by executable tests and harnesses.

The goal is not to redesign ROS.
It is to **codify existing semantic reality** from the reference implementation and make it testable.

---

## The Three-Layer Architecture

We organize specifications into three distinct layers of abstraction to separate "Physics" from "Protocol" from "Operation."

### 1. Core Contracts ("The Physics")
**State machine invariants enforced by the engine.**
These define the internal logic that must hold true regardless of the transport layer.
* *Example:* "A Goal ID must never be reused within a session."
* *Example:* "Lifecycle transitions must be sequential and atomic."
* [View Core Specs](docs/spec/core/)

### 2. Global Specifications ("The Wire")
**Observable behavior over DDS topics and services.**
These define how the internal state is projected to the outside world.
* *Example:* "Lifecycle transitions must emit a `TransitionEvent` message."
* *Example:* "Action servers must expose `/_action/send_goal` service."
* [View Global Specs](docs/spec/global/)

### 3. System Contracts ("The Runtime")
**Operational constraints for production stability.**
These are informed by ecosystem usage (e.g., Nav2, orchestration) and validated against the baseline reference implementation.
* *Example:* "Nodes must clean up resources on shutdown within timeout bounds."
* *Example:* "Executor callbacks must not block indefinitely."
* [View System Specs](docs/spec/system/)

---

## The Oracle Methodology

All contracts in this repository are defined against an explicit upstream baseline.
We do not guess at correctness; we measure it using a **Comparative Physics** approach.

**Baseline Semantic Anchor:** `ROS 2 Jazzy + rclcpp`  
**Ecosystem Stress Tests:** `Nav2` (evidence amplifier, not authority)

Upstream ROS documentation and REPs are authoritative where they are complete.
Where they are silent or ambiguous, the behavior of this production stack defines the semantic truth for the purposes of this repository.

**Ecosystem consumers are used to discover hidden invariants, not to define them.**

### Validation Process

1. **Hypothesis:** A behavior is observed in production systems.
2. **Inventory:** It is logged in [Missing Semantics](docs/provenance/missing_semantics.md).
3. **Codification:** It is written into a Spec (marked `UNVALIDATED`).
4. **Execution:** The [Oracle Harness](harness/) runs a scenario against the **Baseline**.
5. **Validation:**
   * If the Baseline passes, the Spec becomes **Normative**.
   * If a target (e.g., `rclrs`) fails where the Baseline passes, a [Divergence Record](docs/provenance/divergence_records/) is created.

**This is not a competing standard.**
**It is a measurement instrument.**

---

## Why Core Contracts Exist

ROS 2 does not lack documentation, quality processes, or testing culture.
It lacks **canonical semantic contracts** that are:

* Independent of implementation language
* Independent of middleware binding
* Testable against any compliant stack
* Explicit about what remains undefined

The core specifications in this repository define the *minimum semantic truth* that all compliant implementations must share.
From those core contracts, the repository supports:

* **Contract tests** — executable validation of semantic correctness
* **Reference executable semantics** — a runnable model of correct behavior
* **Adapter conformance checks** — explicit verification of cross-language parity
* **Divergence detection** — tracking differences between implementations
* **Regression classification** — distinguishing upstream changes from bugs

Without a core semantic layer, parity and correctness cannot be stated—only guessed.

**We welcome collaboration with upstream maintainers** to ensure these contracts accurately reflect intended behavior and to identify areas where formal specifications would benefit the broader ecosystem.

---

## Validation Roadmap

Our goal is not to force every client library to be identical, but to make differences **explicit and traceable**.

| Phase | Target Stack | Role | Status |
|:------|:------------|:-----|:-------|
| **I** | **ROS 2 Jazzy + rclcpp** | **Baseline Semantic Anchor** | 🟡 In Progress |
| **II** | **rclpy** | Comparison (The Flexible Standard) | ⬜ Planned |
| **III** | **rclrs** | Comparison (The Safety Standard) | ⬜ Planned |
| **IV** | **rclgo / rosrustext_rclrs** | Comparison (Alternative Stacks) | ⬜ Planned |
| **V** | **ROS 2 Humble/Rolling** | Regression Analysis | ⬜ Planned |

---

## Repository Structure

```text
docs/
├── philosophy.md              # How semantic truth is defined and governed
├── intent.md                  # Project goals and roadmap
├── methodology.md             # How we measure semantic truth
├── spec/                      # THE LAW: Normative Contracts
│   ├── core/                  # Physics (State Machines)
│   ├── global/                # Protocol (Wire Format)
│   └── system/                # Operation (Executor/Composition)
└── provenance/                # THE EVIDENCE
    ├── oracle/                # Epistemology (Measurement rules, Scenarios)
    ├── divergence_records/    # Known differences between backends
    └── missing_semantics.md   # Backlog of unvalidated hypotheses

harness/                       # THE JUDGE: Oracle Runners & Scenarios
reference/                     # Reference executable semantics (Rust)
tools/                         # Validation, linting, and support scripts
```

---

## Contributing

This repository treats **rclcpp + Jazzy** as the baseline semantic anchor.

Where this repository's specifications diverge from upstream ROS documentation or REPs, **upstream is authoritative**.
Where upstream is silent, production behavior defines truth.

We welcome:
- Scenarios that expose unspecified behavior
- Divergence reports from alternative implementations
- Clarifications where contracts are ambiguous
- Evidence that contradicts a normative spec (triggers re-measurement)
- Collaboration with upstream maintainers to align specifications

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

---

## License

This repository is licensed under Apache 2.0.
Specifications are intended to be freely implementable without restriction.