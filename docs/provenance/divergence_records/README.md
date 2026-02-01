# Divergence Records

This directory contains the **official record of semantic differences** between the Baseline Authority (Jazzy+rclcpp) and other implementations.

> **Purpose:** To allow engineers to make informed decisions about which client library or distribution fits their strict requirements.

---

## Divergence Classifications

When a backend fails a scenario that the Baseline passes, we classify the divergence:

### 1. Defect (`TYPE: DEFECT`)
The implementation is objectively broken (e.g., it crashes, deadlocks, or violates the core state machine).
* **Action:** File issue upstream. Track fix.

### 2. Missing Feature (`TYPE: GAP`)
The implementation has not yet implemented a spec feature (e.g., "Parameter events not yet supported").
* **Action:** Document limitation. Update `coverage_matrix` to SKIP.

### 3. Intentional Deviation (`TYPE: FORK`)
The implementation explicitly chooses a different behavior for architectural reasons (e.g., "Go client uses channels instead of callbacks").
* **Action:** Document the behavioral difference. The Spec may need a "Variant" clause if this behavior is valid.

### 4. Baseline Regression (`TYPE: REGRESSION`)
A newer version (e.g., Rolling) broke a behavior present in the Baseline (Jazzy).
* **Action:** Critical alert. This breaks ecosystem stability.

---

## Record Format

Divergence records are immutable Markdown files named `DIV_<ID>_<Backend>_<ShortDesc>.md`.

**Example:** `DIV_001_rclrs_parameter_atomicity_gap.md`