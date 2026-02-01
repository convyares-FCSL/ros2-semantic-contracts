# Oracle Validation Plan

This document tracks the roadmap for validating **Hypothetical Requirements**.
Requirements marked `UNVALIDATED (baseline hypothesis)` in the specs must be proven by this plan.

---

## 1. Validation Priority Queue

### Priority A: Core State Machines (Physics)
*Goal: Prove the engine behaves as defined.*

* **Action Core (`SPEC_AC**`)**
    * [ ] `A15_no_goal_id_reuse`: Prove UUIDs are never recycled.
    * [ ] `A03_unknown_not_persistent`: Prove unknown goals vanish.
    * [ ] `A06_status_updates_monotonic`: Prove state never regresses.

* **Lifecycle Core (`SPEC_LC**`)**
    * [ ] `L03_intermediate_states_hidden`: Prove `GetState` never shows `Configuring`.
    * [ ] `L06_error_processing_deterministic`: Prove error routing (Success->Unconfigured).

### Priority B: System Interoperability (BIC)
*Goal: Prove the ecosystem requires these constraints.*

* **System Contract (`SPEC_SYS**`)**
    * [ ] `S17_no_hidden_execution`: Prove no threads spin without `spin()`.
    * [ ] `S19_graph_hygiene`: Prove clean teardown (no orphans).
    * [ ] `A17_abandoned_goal_cleanup`: Prove resource hygiene (GC).

### Priority C: Global Protocol (Wire)
*Goal: Prove observability matches expectations.*

* **Global Specs**
    * [ ] `P07_change_record_ordering`: Prove parameter events are ordered.
    * [ ] `A12_supersession_cancelled`: Prove preemption results in `CANCELED`.
    * [ ] `L12_services_not_gated`: Prove custom services bypass lifecycle.

---

## 2. Execution Strategy

For each item above:

1.  **Harness Implementation:** Create a `stub` backend test case.
2.  **Baseline Capture:** Run against `ros` (Jazzy) backend.
    * *If PASS:* Mark Spec as `VALIDATED (Sxx)`.
    * *If FAIL:* Investigate divergence (Bug vs Spec Error).
3.  **Target Verification:** Run against `rclrs` backend.

---

## 3. Status Tracking

| Spec ID | Scenario | Status | Trace Ref |
| :--- | :--- | :--- | :--- |
| `SPEC_AC02` | `A15` | ⬜ Planned | - |
| `SPEC_ECO03` | `A12` | ⬜ Planned | - |
| `SPEC_LC05` | `L06` | ⬜ Planned | - |
| `SPEC_PC07` | `P07` | ⬜ Planned | - |

*(Legend: ⬜ Planned, 🟡 In Progress, 🟢 Validated, 🔴 Divergence)*