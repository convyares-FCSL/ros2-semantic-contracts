# Global Scenario Semantics (G)

This document defines the **semantic claims** made by Global scenarios.
These scenarios typically span multiple subsystems or enforce generic node behaviors.

It answers the question:
> **What does scenario G (gate) assert about the observable world?**

Each entry defines:
* the **semantic invariant** being exercised
* what **must be observable**
* what **must not** be inferred (Non-claims)

---

## **Conventions**
* **Normative**: derived from top-level ROS 2 specifications or core semantic contracts.
* **Baseline hypothesis**: expected behaviour inferred from implementation/ecosystem, not yet validated.
* **Policy-layer**: behaviour dependent on higher-level orchestration (e.g. Nav2).

---

### G01 — Inactive Suppresses Pubs
**Validates:** `SPEC_L07` (Lifecycle Global)
**Layer:** Core

**Claim**
When a lifecycle node is `Inactive`, it MUST suppress publication on managed publishers.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- No messages appear on the topic while the node is `Inactive`.
- Messages resume (or start) only when the node transitions to `Active`.

**Non-claims**
- Does not define the method by which the node is set to inactive (manual vs managed).
</details>