# Contributing to ROS 2 Semantic Contracts

Thank you for your interest in contributing to this repository.

This project has a specific and unusual structure: it is **measurement-driven**, not design-driven. Contributions must align with the oracle methodology and the principle that **upstream is authoritative**.

---

## Quick Start

Before contributing, please read:
- [docs/philosophy.md](docs/philosophy.md) — How semantic truth is defined
- [docs/intent.md](docs/intent.md) — What this repository is and is not

If you're reporting a divergence or proposing a new contract, start here:
- [docs/provenance/missing_semantics.md](docs/provenance/missing_semantics.md) — Backlog of unvalidated hypotheses

---

## What We Accept

### ✅ We Welcome

1. **Divergence Reports**
   - Evidence that an implementation (rclpy, rclrs, etc.) behaves differently than rclcpp + Jazzy
   - Must include: reproduction steps, observed behavior, expected behavior per baseline
   - Creates a [Divergence Record](docs/provenance/divergence_records/)

2. **Missing Semantics**
   - Observations of behavior that is not yet specified
   - Must include: where you observed it, what stack, and why it matters
   - Logged in [docs/provenance/missing_semantics.md](docs/provenance/missing_semantics.md)
   - Does NOT immediately become a spec (must go through oracle validation)

3. **Oracle Scenarios**
   - Executable test cases for the harness
   - Must be runnable against the baseline (Jazzy + rclcpp)
   - See [harness/README.md](harness/README.md) for format

4. **Evidence That Contradicts a Normative Spec**
   - If you have proof that a `NORMATIVE` spec is wrong, we want to know
   - Triggers re-measurement against the baseline
   - May result in spec correction or divergence record

5. **Clarifications**
   - If a spec is ambiguous or unclear, propose better wording
   - Must preserve semantic intent

6. **Documentation Improvements**
   - Typos, broken links, unclear explanations
   - Philosophy and methodology clarifications

---

## What We Do Not Accept

### ❌ We Cannot Accept

1. **Design Proposals for New ROS Behavior**
   - This repository does not design ROS semantics
   - It measures existing behavior
   - Proposals for changing ROS itself belong in [ROS 2 discussions](https://discourse.ros.org/)

2. **Specs Without Oracle Validation**
   - A spec cannot become `NORMATIVE` without passing the baseline
   - All new contracts must start as `UNVALIDATED` hypotheses

3. **Subjective "Best Practices"**
   - System Contracts must be observable and testable
   - "Should use reasonable timeouts" is not a contract
   - "Must complete shutdown within 5 seconds" is a contract

4. **Breaking Changes to Existing Normative Specs**
   - Once a spec is validated and marked `NORMATIVE`, changes require:
     - Evidence that the baseline has changed (upstream update)
     - OR proof that the original measurement was wrong
   - Casual "improvements" are not accepted

5. **Forks of Upstream ROS Documentation**
   - This is not a replacement for official ROS docs
   - Where upstream docs are authoritative, link to them
   - Do not copy/paste REPs or official docs

---

## Contribution Workflow

### For Divergence Reports

1. **Check Existing Records**
   - Search [docs/provenance/divergence_records/](docs/provenance/divergence_records/)
   - If it already exists, add your evidence there

2. **Create a New Report** (if needed)