# Backend Coverage Matrix

This document records the **observed and implemented semantic coverage** across backends.

It answers the question:
> **Which scenarios have been implemented and satisfied by which backend?**

It is **not inferred** from backend self-reporting and is **not derived automatically** from traces. It is a maintained record of what has actually been implemented, exercised, and judged by the oracle.

---

<details open>
<summary><strong>Legend and Status Codes</strong></summary>

* **PASS** — Scenario implemented and expectations satisfied.
* **FAIL** — Scenario implemented and expectations violated.
* **SKIP** — Scenario executed but not judgeable yet (no expectations defined).
* **N/A** — Scenario not implemented for this backend.
</details>

<details>
<summary><strong>Backend Definitions</strong></summary>

* **`stub`** — Minimal harness validation backend.
* **`ros`** — ORACLE_A (Jazzy + rclcpp baseline witness).
* **`rclrs`** — Rclrs-based ROS 2 interaction (verification target).
* **`PROD`** — Production stack (HyQube/HyFlow/HyFleet).
</details>

---

## Coverage Matrix by Scenario Family

Section summaries are row-level scenario outcomes based on current backend cells.

<details>
<summary><strong>Actions (A): 17 scenarios | PASS: 4 | SKIP: 2 | N/A-only: 11</strong></summary>

| Spec ID | Scenario ID                      | stub | ros  | PROD | PY   | rclrs | Notes                                                     |
| ------- | -------------------------------- | ---- | ---- | ---- | ---- | ----- | --------------------------------------------------------- |
| **A01** | A01_no_ghosts_after_rejection    | PASS | N/A  | PASS | N/A  | N/A   | Action goal identity                                      |
| **A02** | A02_terminal_immutability        | PASS | N/A  | PASS | N/A  | N/A   | Terminal immutability                                     |
| **A03** | A03_unknown_not_persistent       | N/A  | SKIP | N/A  | N/A  | N/A   | Not implemented yet                                       |
| **A04** | A04_valid_transitions            | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A05** | A05_invalid_transitions_rejected | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A06** | A06_status_updates_monotonic     | N/A  | N/A  | PASS | N/A  | N/A   | Status updates monotonic (seq checks)                     |
| **A07** | A07_ordering_time_independent    | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A08** | A08_cancel_intent_observable     | N/A  | N/A  | PASS | N/A  | N/A   | Cancel service interaction + STATUS_CANCELING observable  |
| **A09** | A09_cancel_resolution_policy     | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A10** | A10_result_visible_after_terminal | N/A | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A11** | A11_single_terminal_result       | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A12** | A12_supersession_cancelled       | N/A  | SKIP | N/A  | N/A  | N/A   | Policy-layer behavior, unvalidated                        |
| **A13** | A13_result_retention             | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A14** | A14_tooling_compatibility        | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A15** | A15_no_goal_id_reuse             | N/A  | N/A  | N/A  | N/A  | N/A   | ⚠️ Deferred — adversarial UUID-reuse stimulus not expressible in current ops model |
| **A16** | A16_action_interface_conformity  | N/A  | N/A  | N/A  | N/A  | N/A   | **New**: Interface Conformity                             |
| **A17** | A17_abandoned_goal_cleanup       | N/A  | N/A  | N/A  | N/A  | N/A   | **New**: Resource hygiene                                 |
</details>

<details>
<summary><strong>Parameters (P): 16 scenarios | PASS: 15 | N/A-only: 1</strong></summary>

| Spec ID | Scenario ID                      | stub | ros | PROD | PY   | rclrs | Notes                                       |
| ------- | -------------------------------- | ---- | --- | ---- | ---- | ----- | ------------------------------------------- |
| **P01** | P01_no_fabricated_params         | N/A  | N/A | PASS | PASS | N/A   | Unknown/case-mismatch names remain undeclared (NOT_SET) |
| **P02** | P02_dynamic_typing_rules         | N/A  | N/A | PASS | PASS | N/A   | Static rejects type change; dynamic allows it |
| **P03** | P03_redeclaration_fails          | N/A  | N/A | PASS | PASS | N/A   | Redeclaration of parameter fails            |
| **P04** | P04_no_partial_application       | N/A  | N/A | PASS | PASS | N/A   | Atomic batch-set rejection                  |
| **P05** | P05_readonly_enforcement         | N/A  | N/A | PASS | PASS | N/A   | Read-only enforcement                       |
| **P06** | P06_unknown_params_rejected      | N/A  | N/A | PASS | PASS | N/A   | Set undeclared param rejected               |
| **P07** | P07_change_record_ordering       | N/A  | N/A | PASS | PASS | N/A   | Sequential writes observed in deterministic order |
| **P08** | P08_unknown_types_not_set        | N/A  | N/A | PASS | PASS | N/A   | Undeclared param returns NOT_SET            |
| **P09** | P09_atomic_updates               | N/A  | N/A | PASS | PASS | N/A   | Mixed batch failure shows no externally visible partial update |
| **P10** | P10_rejected_updates_silent      | N/A  | N/A | PASS | PASS | N/A   | Proxy: rejected set leaves no observable state change |
| **P11** | P11_events_describe_changes      | N/A  | N/A | PASS | PASS | N/A   | Proxy: successful updates yield observable value/type transitions |
| **P12** | P12_describe_unknown_consistent  | N/A  | N/A | PASS | PASS | N/A   | Undeclared param returns consistent default |
| **P13** | P13_list_complete_stable         | N/A  | N/A | PASS | PASS | N/A   | Proxy: repeated reads over declared set remain complete/stable |
| **P14** | P14_deletion_reported_events     | N/A  | N/A | PASS | PASS | N/A   | Proxy: dynamic-typed undeclare removes parameter from observable state |
| **P15** | P15_parameter_interface_conformity | N/A | N/A | N/A  | N/A  | N/A   | **New**: Interface Conformity               |
| **P16** | P16_explicit_declaration_mode    | N/A  | N/A | PASS | PASS | N/A   | Strict declaration mode enforced (undeclared set rejected) |
</details>

<details>
<summary><strong>Harness (H): 2 scenarios | PASS: 2</strong></summary>

| Spec ID | Scenario ID          | stub | ros  | PROD | PY   | rclrs | Notes                                   |
| ------- | -------------------- | ---- | ---- | ---- | ---- | ----- | --------------------------------------- |
| **H00** | H00_harness_smoke    | PASS | PASS | PASS | PASS | N/A   | Harness calibration (action smoke test) |
| **H01** | H01_ros_clock_smoke  | PASS | PASS | PASS | PASS | N/A   | ROS clock API smoke test                |
</details>

<details>
<summary><strong>Lifecycle (L): 13 scenarios | N/A-only: 13</strong></summary>

| Spec ID | Scenario ID                        | stub | ros | PROD | PY  | rclrs | Notes                              |
| ------- | ---------------------------------- | ---- | --- | ---- | --- | ----- | ---------------------------------- |
| **L01** | L01_defined_transitions            | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L02** | L02_primary_state_visibility       | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L03** | L03_intermediate_states_hidden     | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L04** | L04_single_active_transition       | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L05** | L05_concurrent_transition_rejected | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L06** | L06_error_processing_deterministic | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L07** | L07_get_state_primary              | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L08** | L08_rejection_observable           | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L09** | L09_transition_event_emitted       | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L10** | L10_shutdown_deterministic         | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                  |
| **L11** | L11_shutdown_via_deactivate        | N/A  | N/A | N/A  | N/A | N/A   | **New**: Active shutdown logic     |
| **L12** | L12_services_not_gated             | N/A  | N/A | N/A  | N/A | N/A   | **New**: Service availability      |
| **L13** | L13_lifecycle_interface_conformity | N/A  | N/A | N/A  | N/A | N/A   | **New**: Interface Conformity      |
</details>

<details>
<summary><strong>Global (G): 1 scenario | N/A-only: 1</strong></summary>

| Spec ID | Scenario ID                  | stub | ros | PROD | PY  | rclrs | Notes     |
| ------- | ---------------------------- | ---- | --- | ---- | --- | ----- | --------- |
| **G01** | G01_inactive_suppresses_pubs | N/A  | N/A | N/A  | N/A | N/A   | Gate rule |
</details>

<details>
<summary><strong>System (S): 20 scenarios | N/A-only: 20</strong></summary>

| Spec ID | Scenario ID                         | stub | ros | PROD | PY  | rclrs | Notes                                                  |
| ------- | ----------------------------------- | ---- | --- | ---- | --- | ----- | ------------------------------------------------------ |
| **S01** | S01_load_node_observable            | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S02** | S02_load_failure_clean              | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S03** | S03_unload_node_destroyed           | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S04** | S04_unique_node_ids                 | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S05** | S05_concurrent_load_deterministic   | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S06** | S06_remaps_apply_load               | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S07** | S07_explicit_ownership              | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S08** | S08_bounded_spin_modes              | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S09** | S09_shutdown_bounded                | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S10** | S10_callback_failure_visible        | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S11** | S11_lifecycle_executor_safety       | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S12** | S12_lifecycle_orchestration         | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S13** | S13_liveness_detection              | N/A  | N/A | N/A  | N/A | N/A   | ⚠️ Deferred — requires kill-node or adversarial timeout primitive |
| **S14** | S14_tooling_interoperability        | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S15** | S15_explicit_ownership_lifetime     | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S16** | S16_deterministic_bringup           | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S17** | S17_no_hidden_execution             | N/A  | N/A | N/A  | N/A | N/A   | ⚠️ Manual/deferred — not automatable; requires static analysis |
| **S18** | S18_observable_failure              | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S19** | S19_graph_hygiene                   | N/A  | N/A | N/A  | N/A | N/A   | Not yet validated                                      |
| **S20** | S20_composition_interface_conformity | N/A | N/A | N/A  | N/A | N/A   | Interface Conformity                                   |
</details>
