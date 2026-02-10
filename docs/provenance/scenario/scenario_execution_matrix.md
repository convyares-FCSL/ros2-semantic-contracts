# Backend Coverage Matrix

This document records the **observed and implemented semantic coverage** across backends.

It answers the question:
> **Which scenarios have been implemented and satisfied by which backend?**

It is **not inferred** from backend self-reporting and is **not derived automatically** from traces. It is a maintained record of what has actually been implemented, exercised, and judged by the oracle.

---

<details open>
<summary><strong>Legend and Status Codes</strong></summary>

* **✅** — Scenario implemented and expectations satisfied.
* **❌** — Scenario implemented and expectations violated.
* **⚠️** — Scenario executed but not judgeable yet (no expectations defined).
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
<summary><strong>Actions (A): 17 scenarios | ✅: 4 | ⚠️: 2 | N/A-only: 11</strong></summary>

| Spec ID | Scenario ID                      | stub | ros  | PROD | PY   | rclrs | Notes                                                     |
| ------- | -------------------------------- | ---- | ---- | ---- | ---- | ----- | --------------------------------------------------------- |
| **A01** | A01_no_ghosts_after_rejection    | ✅ | N/A  | ✅ | N/A  | N/A   | Action goal identity                                      |
| **A02** | A02_terminal_immutability        | ✅ | N/A  | ✅ | N/A  | N/A   | Terminal immutability                                     |
| **A03** | A03_unknown_not_persistent       | N/A  | ⚠️ | N/A  | N/A  | N/A   | Not implemented yet                                       |
| **A04** | A04_valid_transitions            | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A05** | A05_invalid_transitions_rejected | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A06** | A06_status_updates_monotonic     | N/A  | N/A  | ✅ | N/A  | N/A   | Status updates monotonic (seq checks)                     |
| **A07** | A07_ordering_time_independent    | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A08** | A08_cancel_intent_observable     | N/A  | N/A  | ✅ | N/A  | N/A   | Cancel service interaction + STATUS_CANCELING observable  |
| **A09** | A09_cancel_resolution_policy     | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A10** | A10_result_visible_after_terminal | N/A | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A11** | A11_single_terminal_result       | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A12** | A12_supersession_cancelled       | N/A  | ⚠️ | N/A  | N/A  | N/A   | Policy-layer behavior, unvalidated                        |
| **A13** | A13_result_retention             | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A14** | A14_tooling_compatibility        | N/A  | N/A  | N/A  | N/A  | N/A   | Not yet validated                                         |
| **A15** | A15_no_goal_id_reuse             | N/A  | N/A  | N/A  | N/A  | N/A   | ⚠️ Deferred — adversarial UUID-reuse stimulus not expressible in current ops model |
| **A16** | A16_action_interface_conformity  | N/A  | N/A  | N/A  | N/A  | N/A   | **New**: Interface Conformity                             |
| **A17** | A17_abandoned_goal_cleanup       | N/A  | N/A  | N/A  | N/A  | N/A   | **New**: Resource hygiene                                 |
</details>

</bk></bk>
<details>
<summary><strong>Parameters (P): 37 scenarios | ✅: 33 | ❌: 4 | N/A-only: 0</strong></summary>

| Spec ID | Scenario ID                      | stub | ros | PROD | PY   | rclrs | Notes                                       |
| ------- | -------------------------------- | ---- | --- | ---- | ---- | ----- | ------------------------------------------- |
| **P01** | P01_no_fabricated_params         | N/A  | N/A | ✅ | ✅ | N/A   | Unknown/case-mismatch names remain undeclared (NOT_SET) |
| **P02** | P02_dynamic_typing_rules         | N/A  | N/A | ✅ | ✅ | N/A   | Static rejects type change; dynamic allows it |
| **P03** | P03_redeclaration_fails          | N/A  | N/A | ✅ | ✅ | N/A   | Redeclaration of parameter fails            |
| **P04** | P04_no_partial_application       | N/A  | N/A | ✅ | ✅ | N/A   | Atomic batch-set rejection                  |
| **P05** | P05_readonly_enforcement         | N/A  | N/A | ✅ | ✅ | N/A   | Read-only enforcement                       |
| **P06** | P06_unknown_params_rejected      | N/A  | N/A | ✅ | ✅ | N/A   | Set undeclared param rejected               |
| **P07** | P07_change_record_ordering       | N/A  | N/A | ✅ | ✅ | N/A   | Sequential writes observed in deterministic order |
| **P08** | P08_unknown_types_not_set        | N/A  | N/A | ✅ | ✅ | N/A   | Undeclared param returns NOT_SET            |
| **P09** | P09_atomic_updates               | N/A  | N/A | ✅ | ✅ | N/A   | Mixed batch failure shows no externally visible partial update |
| **P10** | P10_rejected_updates_silent      | N/A  | N/A | ✅ | ✅ | N/A   | Direct: rejected update emits no `parameter_event` in observation window and state remains unchanged |
| **P11** | P11_events_describe_changes      | N/A  | N/A | ✅ | ✅ | N/A   | Direct: `parameter_event` payloads reflect successful value/type transitions |
| **P12** | P12_describe_unknown_consistent  | N/A  | N/A | ✅ | ✅ | N/A   | Undeclared param returns consistent default |
| **P13** | P13_list_complete_stable         | N/A  | N/A | ✅ | ✅ | N/A   | Direct: `list_params` prefix/depth filtering is complete and stable across repeated calls |
| **P14** | P14_deletion_reported_events     | N/A  | N/A | ✅ | ✅ | N/A   | Direct (Jazzy observed): undeclare removes parameter state; no declare/undeclare `parameter_event` observed |
| **P15** | P15_parameter_interface_conformity | N/A | N/A | ✅ | ✅ | N/A   | Proxy: standard parameter ops are executable and coherent end-to-end |
| **P16** | P16_explicit_declaration_mode    | N/A  | N/A | ✅ | ✅ | N/A   | Strict declaration mode enforced (undeclared set rejected) |
| **P17** | P17_non_atomic_batch_callback_rejection | N/A | N/A | ✅ | ✅ | N/A | Direct: non-atomic batch partially applies under callback rejection (`a=a1`, rejected `b` remains `b0`) |
| **P18** | P18_atomic_batch_callback_rejection | N/A | N/A | ✅ | ✅ | N/A | Direct: atomic batch rejection preserves rollback (`a=a0`, `b=b0`) |
| **P19** | P19_pre_set_removal_semantics    | N/A  | N/A | ✅ | ✅ | N/A   | Direct discovery: pre-set callback can remove parameters from atomic batch input |
| **P20** | P20_parameter_event_cardinality_batch | N/A | N/A | ✅ | ✅ | N/A | Direct signature: atomic batch emits 1 grouped `parameter_event`; non-atomic batch emits 3 per-parameter events |
| **P21** | P21_descriptor_tracks_value_type | N/A  | N/A | ✅ | ✅ | N/A   | Direct discovery: for dynamic-typed params, descriptor `type` tracks current value type after successful set |
| **P22** | P22_explicit_descriptor_type_enforcement | N/A | N/A | ✅ | ✅ | N/A | Limited: matching value+descriptor type case passes; this does not prove mismatch enforcement |
| **P23** | P23_declare_without_default      | N/A  | N/A | ✅ | ✅ | N/A   | Direct discovery: static typed declare without default succeeds and reads as `NOT_SET` until initialized |
| **P24** | P24_get_parameter_types_response_length | N/A | N/A | ❌ | ❌ | N/A | Official-contract conformance check: mixed declared/undeclared query should return 1:1 types array; both backends collapse to `NotDeclared` with empty response (same in pinned Docker Jazzy+Humble) |
| **P25** | P25_concurrent_visibility_probe  | N/A  | N/A | ✅ | ❌ | N/A   | Repeated single-call non-atomic concurrency probe: `prod` observed mixed snapshots (`partial_state_observed=true`), `py` observed no mixed snapshots (`false`); same asymmetry in pinned Docker Jazzy+Humble |
| **P26** | P26_type_coercion_matrix         | N/A  | N/A | ✅ | ✅ | N/A   | Direct non-happy-path matrix: static params reject cross-type scalar sets (no implicit coercion on tested paths); dynamic control transitions succeed |
| **P27** | P27_standard_batch_partial_application_check | N/A | N/A | ✅ | ✅ | N/A | Direct: non-atomic batch keeps earlier success when later key is callback-rejected (same in pinned Docker Jazzy+Humble) |
| **P28** | P28_atomic_batch_transaction_check | N/A | N/A | ✅ | ✅ | N/A | Direct: atomic batch callback rejection preserves all pre-existing values (same in pinned Docker Jazzy+Humble) |
| **P29** | P29_callback_storage_old_value_visibility | N/A | N/A | ✅ | ✅ | N/A | Direct: validation callback can assert storage still shows old committed value during set (same in pinned Docker Jazzy+Humble) |
| **P30** | P30_external_visibility_after_success_only | N/A | N/A | ✅ | ✅ | N/A | Direct: rejected batch emits no event and successful follow-up emits committed change event (same in pinned Docker Jazzy+Humble) |
| **P31** | P31_no_default_with_startup_override | N/A | N/A | ✅ | ❌ | N/A | Exact CLI path (`--ros-args -p`) for typed no-default declaration: ✅ on prod and py(Humble), ❌ on py(Jazzy) with `PARAMETER_NOT_SET` declaration error |
| **P32** | P32_startup_declare_event_batching | N/A | N/A | ❌ | ❌ | N/A | Startup `declare_parameter` batching expectation violated (`event_count=3`, `max_keys_in_single_event=1`) on both backends in pinned Docker Jazzy+Humble |
| **P33** | P33_standard_batch_partial_application_type_failure | N/A | N/A | ✅ | ✅ | N/A | Result-first standard profile: later type failure does not roll back earlier success (`a=11`, `b=20`) (same in pinned Docker Jazzy+Humble) |
| **P34** | P34_atomic_batch_transaction_integrity_type_failure | N/A | N/A | ✅ | ✅ | N/A | Result-first atomic profile: mixed valid/invalid batch rejects as unit and preserves both values (same in pinned Docker Jazzy+Humble) |
| **P35** | P35_concurrent_visibility_atomic_control | N/A | N/A | ✅ | ✅ | N/A | Repeated single-call atomic concurrency control: no mixed snapshots observed (`partial_state_observed=false`) in both backends (same in pinned Docker Jazzy+Humble) |
| **P36** | P36_integer_descriptor_range_enforcement | N/A | N/A | ✅ | ✅ | N/A | Descriptor integer range is enforced on set; out-of-range write is rejected (same in pinned Docker Jazzy+Humble) |
| **P37** | P37_floating_descriptor_range_enforcement | N/A | N/A | ✅ | ✅ | N/A | Descriptor floating-point range is enforced on set; out-of-range write is rejected (same in pinned Docker Jazzy+Humble) |
</details>

</bk></bk>
<details>
<summary><strong>Harness (H): 2 scenarios | ✅: 2</strong></summary>

| Spec ID | Scenario ID          | stub | ros  | PROD | PY   | rclrs | Notes                                   |
| ------- | -------------------- | ---- | ---- | ---- | ---- | ----- | --------------------------------------- |
| **H00** | H00_harness_smoke    | ✅ | ✅ | ✅ | ✅ | N/A   | Harness calibration (action smoke test) |
| **H01** | H01_ros_clock_smoke  | ✅ | ✅ | ✅ | ✅ | N/A   | ROS clock API smoke test                |
</details>

</bk></bk>
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

</bk></bk>
<details>
<summary><strong>Global (G): 1 scenario | N/A-only: 1</strong></summary>

| Spec ID | Scenario ID                  | stub | ros | PROD | PY  | rclrs | Notes     |
| ------- | ---------------------------- | ---- | --- | ---- | --- | ----- | --------- |
| **G01** | G01_inactive_suppresses_pubs | N/A  | N/A | N/A  | N/A | N/A   | Gate rule |
</details>

</bk></bk>
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
