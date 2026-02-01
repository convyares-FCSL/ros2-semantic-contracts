# Harness Scenario Semantics (H)

This document defines the **semantic claims** made by Harness self-test scenarios.

---

### H00 — Behavioral Indistinguishability

**Validates:** `SPEC_ECO05` (System)
**Layer:** System

**Claim**
A compliant backend must be behaviorally indistinguishable from the baseline (Jazzy + rclcpp) when observed through the harness.

<details>
<summary>Assertions and Non-claims</summary>

**Observable assertions**
- The harness can run against both the baseline and a candidate backend.
- Trace events from both are structurally comparable.
- Verdicts can be computed by comparing observable outputs.

**Non-claims**
- Does not assert internal implementation details match.
- Does not require identical timing characteristics.
</details>
