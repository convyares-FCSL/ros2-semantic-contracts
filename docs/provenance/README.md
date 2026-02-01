# Provenance & Oracle Methodology

This directory contains the **epistemological foundation** of the Semantic Contracts.
It defines *how* we measure truth, *what* we measure against, and the *evidence* that supports our claims.

> **Provenance:** The chronology of the ownership, custody, or location of a historical object.
> In this repo: The chain of evidence linking a Spec Requirement to a physical observation.

---

## Directory Structure

### 1. The Oracle (`oracle/`)
Defines the judge and the laws of measurement.
* **`oracle_measurement.md`**: Defines *how* we measure (Observation Windows, Verdicts, Settling Time).
* **`oracle_baseline.md`**: Defines *who* we judge against (The **[R1]** Authority: Jazzy + rclcpp + Nav2).
* **`oracle_naming.md`**: Rules for Spec IDs (`A01`) and Scenario IDs.
* **`oracle_plan.md`**: The roadmap for validating hypothetical requirements (`UNVALIDATED` tags).
* **`scenario/`**: Detailed definitions of test scenarios (e.g., `A01`, `L05`) and the **Coverage Matrix**.

### 2. The Backlog (`missing_semantics.md`)
The inventory of "Known Unknowns."
* Lists behaviors we suspect are true (Hypotheses) but haven't proven yet.
* Acts as the to-do list for the Harness engineering team.

### 3. Evidence Artifacts
* **`divergence_records/`**: Formal records of where specific backends (e.g. `rclrs`, `humble`) violate the Baseline Spec.
* **`trace_records/`**: (Planned) Where raw `rosbag` traces and Oracle judgment logs will be stored.

---

## The Provenance Lifecycle

1.  **Hypothesis**: A requirement is written in `docs/spec/` marked `⚠️ UNVALIDATED`.
2.  **Inventory**: It is listed in `missing_semantics.md`.
3.  **Planning**: A Scenario ID (e.g., `[S99]`) is assigned in `oracle_plan.md`.
4.  **Execution**: The scenario is implemented in the Harness and run against the Baseline.
5.  **Validation**:
    * **PASS**: The tag changes to `✅ VALIDATED (S99)`.
    * **FAIL**: A Divergence Record is created (`DIV_001...`), or the Spec is updated.