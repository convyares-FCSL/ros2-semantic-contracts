# Intent — ros2-semantic-contracts

This repository exists to define, version, and enforce **normative semantic contracts** for ROS 2 core behaviour against explicit upstream baselines.

It establishes a single, explicit definition of *what correct behaviour means* for ROS 2 mechanisms (e.g. lifecycle, actions, parameters) within a defined upstream baseline where upstream documentation defines interfaces and APIs but does not fully specify semantics.

The contracts in this repository are transport-agnostic, language-agnostic, and testable.

---

## What This Repository Is

This repository is:

* A **semantic authority layer** for ROS 2 core behaviour, grounded in upstream specification and production evidence
* A collection of **normative contracts** describing required behaviour and failure responses
* A **baseline definition** derived from a named production profile (initially: ROS 2 Jazzy + rclcpp + Nav2)
* A set of **executable enforcement tools** (reference implementations, contract tests, and harnesses)
* A mechanism to make **semantic drift visible, reviewable, and versioned**

It defines *what must happen* — not *how to implement it*.

---

## What This Repository Is Not

This repository is not:

* A ROS 2 tutorial or learning sequence
* A reference application or demo system
* A best-practices guide
* A performance benchmark suite (except as supporting evidence)
* A replacement for ROS documentation, REPs, or quality processes
* A language advocacy project

It does not teach ROS.
It defines semantic truth.

---

## Who This Repository Is For

This repository primarily serves:

* **Client library implementers** (rclcpp, rclpy, rclrs, rosrustlib, and others)
* **Middleware and tooling authors** who need a precise semantic target
* **System integrators** who must reason about behaviour under failure, race, and orchestration pressure

It is written for engineers who require **predictability, auditability, and testable correctness**, not for onboarding or exploration.

---

## How This Repository Is Used

The repository is used to:

* Determine whether an implementation **conforms to defined semantics**
* Compare observed behaviour against a **named semantic baseline**
* Detect and document divergence across languages, versions, or ecosystems
* Support automated verification via contract tests and oracle harnesses
* Anchor discussions about correctness in **reproducible evidence**, not interpretation

Claims made by this repository are always traceable to:
upstream documentation, upstream code, or reproducible observation.

---

## Relationship to Client Libraries and Ecosystems

This repository does not define APIs.
It defines **required behaviour**.

Where upstream ROS specifications are incomplete, the behaviour of the production stack defines the baseline.
For the initial contract set, **rclcpp behaviour in ROS 2 Jazzy, as exercised by Nav2**, is treated as canonical.

Other client libraries are evaluated relative to this contract.
Deviation is permitted, but it must be **explicitly documented and justified**.

---

## Scope Discipline

Only behaviour that affects **semantic correctness, safety, or liveness** belongs in this repository.

Performance characteristics, tuning guidance, and deployment advice are out of scope unless they directly affect correctness or operability.

This constraint is deliberate.
It keeps the contract set minimal, enforceable, and stable.

---

## Change and Evolution

Semantic contracts evolve through **explicit versioning**, not silent modification.

Each contract set is tied to a named upstream baseline.
Future changes are evaluated as regressions until proven intentional via upstream evidence.

Historical contracts are retained.
At any given time, only one contract set is authoritative.

---

## Closing

This repository exists to make ROS 2 behaviour **explicit, testable, and comparable**.

It does not promise completeness.
It promises clarity.
