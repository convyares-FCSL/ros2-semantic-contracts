Related matrix:
- `docs/spec/spec_scenario_matrix.md` (spec-to-scenario execution status matrix for closure tracking)

### Parameter Core Contracts (`SPEC_PC`)

| ID | Title | Authority Source | Usefulness | Status |
| --- | --- | --- | --- | --- |
| **SPEC_PC01** | Identity & Case Sensitivity | [Official Docs] | **Essential**: Prevents naming collisions. | Required |
| **SPEC_PC02** | Existence States | [Official Docs] | **High**: Defines the state machine logic. | Required |
| **SPEC_PC03** | Type Enforcement | [Official Docs] | **Critical**: Safety invariant for static nodes. | Required |
| **SPEC_PC04** | Declaration Logic | [Official Docs] | **High**: Prevents double-init bugs. | Required |
| **SPEC_PC05** | Atomic Set Semantics | [Official Docs] | **Critical**: Fundamental "Physics" of ROS 2. | Required |
| **SPEC_PC06** | Undeclared Access Logic | [Official Docs] | **High**: Security/Stability boundary. | Required |
| **SPEC_PC07** | Change Record Model | [Baseline (rclcpp)] | **Medium**: Necessary for deterministic replay. | **Bloat?** (No upstream source) |
| **SPEC_PC08** | Deletion Support | [None] | **Low**: Softened to match `rclcpp` gaps. | **Rot Candidate** |
| **SPEC_PC09** | Descriptor Sync | [Baseline (rclcpp)] | **High**: Essential for dynamic UI/Tools. | Should Have |
| **SPEC_PC10** | Explicit Type Enforcement | [Baseline (rclcpp)] | **High**: Hardens declaration intent. | Should Have |
| **SPEC_PC11** | No-Default Declaration | [Baseline (rclcpp)] | **High**: Standard production pattern. | Should Have |
| **SPEC_PC12** | 1:1 Response Map | [IDL Contract] | **Critical**: Strict protocol conformance. | Required (Failing) |
| **SPEC_PC13** | Non-Atomic Batching | [Official Docs] | **High**: Differentiates standard vs atomic. | Required |
| **SPEC_PC14** | Concurrency Visibility | [Baseline (rclcpp)] | **Med**: Defines performance envelope. | Should Have |
| **SPEC_PC15** | Scalar Coercion Matrix | [Baseline (rclcpp)] | **High**: Prevents data corruption. | Required |
| **SPEC_PC16** | Atomicity Hypotheses | [Baseline (rclcpp)] | **High**: Required for Nav2-style safety. | Should Have |
| **SPEC_PC17** | Descriptor Range Enforcement | [Official Docs + Baseline] | **High**: Prevents invalid runtime config writes. | Required |

---

### Global Parameter Specifications (`SPEC_P`)

| ID | Title | Authority Source | Usefulness | Status |
| --- | --- | --- | --- | --- |
| **SPEC_P01** | Standard Services | [Official Docs] | **Critical**: Base interface compliance. | Required |
| **SPEC_P02** | Declaration Mode | [Official Docs] | **High**: Node configuration contract. | Required |
| **SPEC_P03** | Unknown Introspection | [Official Docs] | **High**: Tooling stability. | Required |
| **SPEC_P04** | Atomic Batch (Global) | [Official Docs] | **Critical**: User-facing atomicity. | Required |
| **SPEC_P05** | Event Determinism | [None] | **Low**: High noise, low safety value. | **Bloat Candidate** |
| **SPEC_P06** | Consistent Description | [None] | **Low**: Undefined error handling. | **Rot Candidate** |
| **SPEC_P07** | Stable List Enumeration | [None] | **Med**: Helpful for introspection tools. | Should Have |
| **SPEC_P08** | Optional Deletion | [Official Docs] | **Low**: Unreliable across libraries. | **Rot Candidate** |
| **SPEC_P09** | Non-Atomic Visibility | [Official Docs] | **High**: Explains "partial" states. | Required |
| **SPEC_P10** | Commit-Gated Visibility | [Baseline (rclcpp)] | **High**: Critical for observer safety. | Should Have |

---

### Summary of the "Rot"

* **The Calibration Trap:** `SPEC_PC08` and `SPEC_P08` are "rot" because we changed the language to "SHOULD" or "Optional" simply because `rclcpp` doesn't do it. This hides a deficiency rather than measuring it.
* **The Bloat:** `SPEC_PC07` and `SPEC_P05` attempt to define "deterministic events." Since the official ROS documentation is silent here, we are inventing "truth" that might actually be implementation-defined jitter.
* **The Missing Conformance:** `SPEC_PC12` (1:1 mapping) is the only place we are successfully resisting rot. It stays as a "MUST" because the IDL requires it, correctly showing a **FAIL** on Jazzy.

**Would you like me to now map these "Bloat" vs "Required" categories to the Scenario (P01-P35) list?**

## 1. Action Contracts (SPEC_AC & SPEC_A)

Action contracts are generally high-integrity because they follow the formal ROS 2 Design documents closely. However, "Supersession" is a significant area of semantic drift.

| ID | Title | Authority Source | Usefulness | Status |
| --- | --- | --- | --- | --- |
| **SPEC_AC01** | Unique Identity | [Official Docs] | **Critical**: Prevents cross-goal interference. | Required |
| **SPEC_AC03** | Terminal Immutability | [Official Docs] | **Critical**: Final states must be final. | Required |
| **SPEC_AC04** | Valid Transition Graph | [Official Docs] | **Critical**: Defines Action state machine. | Required |
| **SPEC_AC05** | Monotonic Ordering | [Official Docs] | **High**: Ensures state sequence logic. | Required |
| **SPEC_A03** | Supersession (Preemption) | [Nav2 / Community] | **High**: Critical for continuous navigation. | Should Have |
| **SPEC_A03** (Baseline) | **Supersession as Canceled** | [Baseline (rclcpp)] | **Low**: Calibrated to rclcpp's specific choice. | **Rot Candidate** |
| **SPEC_A05** | Result Retention | [Official Docs] | **High**: Allows for late result retrieval. | Required |
| **SPEC_A09** | Abandoned Goal Cleanup | [Nav2 / Ecosystem] | **High**: Prevents long-term memory leaks. | Should Have |

---

## 2. Lifecycle Contracts (SPEC_LC & SPEC_L)

The Lifecycle stack is the most prone to "Rot" because the gap between the design article and the actual `rclcpp` implementation regarding error handling and intermediate states is large.

| ID | Title | Authority Source | Usefulness | Status |
| --- | --- | --- | --- | --- |
| **SPEC_LC01** | Primary/Auxiliary States | [Official Docs] | **Critical**: Defines the stable state machine. | Required |
| **SPEC_LC02** | Strict Transition Graph | [Official Docs] | **Critical**: Prevents illegal transitions. | Required |
| **SPEC_LC03** | Intermediate Atomicity | [Official Docs] | **High**: Prevents re-entrant transitions. | Required |
| **SPEC_LC05** | **Error Resolution Logic** | [Baseline (rclcpp)] | **Low**: Calibrated to rclcpp return codes. | **Rot Candidate** |
| **SPEC_L02** | Primary State Visibility | [Official Docs] | **High**: Ensures tools see stable states. | Required |
| **SPEC_L04** | **Transition Event Emission** | [Baseline (rclcpp)] | **Med**: Undocumented but used by observers. | **Bloat?** |
| **SPEC_L06** | Shutdown via Deactivate | [Baseline (rclcpp)] | **High**: Safety requirement for teardown. | Should Have |
| **SPEC_L08** | Service Gating (Negative) | [Baseline (rclcpp)] | **Med**: Warns that services stay active. | Should Have |

---

## 3. System, Executor, & Composition Contracts

These contracts mostly consist of **Baseline Interoperability Constraints (BICs)**. They are "Should Haves" that fix the lack of upstream specifications for production-grade reliability.

| ID | Title | Authority Source | Usefulness | Status |
| --- | --- | --- | --- | --- |
| **SPEC_E01** | Explicit Ownership | [BIC / Community] | **Critical**: Forbids hidden background threads. | Required |
| **SPEC_E03** | Bounded Shutdown | [BIC / Community] | **Critical**: Ensures nodes actually die. | Required |
| **SPEC_SYS01** | Ownership & Lifetime | [BIC / Community] | **High**: Prevents implicit global state. | Required |
| **SPEC_SYS05** | **Graph Hygiene** | [None] | **Low**: Often fails due to RMW race conditions. | **Rot Candidate** |
| **SPEC_C02** | Atomic Load Semantics | [BIC / Community] | **High**: Prevents "ghost" nodes in containers. | Required |
| **SPEC_C04** | Node Identity No-Reuse | [Official Docs] | **High**: Stability for log/graph analysis. | Required |
| **SPEC_C05** | **Concurrency Rejection** | [None] | **Low**: Measures implementation busy-logic. | **Bloat?** |

---

## 4. Audit Diagnosis: The "Rot" Summary

### The "Calibration Trap" (Action/Lifecycle)

* **SPEC_A03 (Supersession):** Upstream is silent on which state to use for preemption. We've mandated `CANCELED` because `rclcpp` does it. This is **Rot** because it prevents other libraries from using a more descriptive custom state even if they follow the "physics" of the Action design.
* **SPEC_LC05 (Error Resolution):** The mapping of `SUCCESS` to `Unconfigured` during an error is purely an `rclcpp` convention. Mandating this as a "Core Contract" forces all other languages (Python, Rust) to mimic C++ internals rather than ROS 2 specs.

### The "Incidental Bloat" (Global/System)

* **SPEC_L04 (Event Emission):** While `rclcpp` emits events for every attempt, a compliant library might only emit them for successful state changes. Mandating attempt-level events is **Bloat** based on a specific implementation's verbosity.
* **SPEC_SYS05 (Graph Hygiene):** This spec often fails because DDS takes time to "forget" nodes. By keeping it as a normative requirement, we are effectively testing the middleware (DDS) speed, not the AI/Client Library's semantic correctness.

### The "Ecosystem Fixes" (The Gold Standard)

* **SPEC_E01 & SPEC_E03:** These are the "professional-grade" additions. They are not in the official docs, but without them, you cannot build a safety-critical system. These are **Essential "Should Haves"**.

Would you like me to create the **Divergence Records** for the "Rot" items (SPEC_A03, SPEC_LC05) so we can move them out of the Core Specs and into Implementation-Specific records?
