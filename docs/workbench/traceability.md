# Semantic Traceability Matrix

This file maps each spec item to:
- provenance (top-level / community / none)
- oracle coverage (scenario IDs)
- production-stack traceability (where it is exercised)

> **Note**: Gate semantics (Gxx) are tracked separately from lifecycle invariants.

Legend:
- Provenance:
  - TL = top-level ROS 2 / core docs (authoritative)
  - COM = community (Nav2, discourse, issues; informative)
  - NONE = not sourced yet (requires oracle evidence or future sourcing)
- Coverage:
  - ORACLE_A = mechanistic (rclcpp-only)
  - ORACLE_B = system (Nav2)
  - PROD = production stack (HyQube/HyFlow/HyFleet etc.)
- Status: Planned / Implemented / Verified

| Spec ID | Layer | Spec doc | Clause / invariant | Provenance | Oracle scenario(s) | ORACLE_A | ORACLE_B | PROD | Notes |
|---|---|---|---|---|---|---|---|---|---|
| A01 | Core | docs/spec/core/action_core.md | Rejected goals are not observable ('no ghosts') | TL | A01_unique_goal_identity | Planned | Planned | Planned | |
| A02 | Core | docs/spec/core/action_core.md | Terminal states immutable | TL | A02_terminal_immutable | Planned | Planned | Planned | |
| A03 | Core | docs/spec/core/action_core.md | Unknown not persistent | TL | A03_unknown_not_persistent | Planned | Planned | Planned | |
| A04 | Core | docs/spec/core/action_core.md | Valid transitions only | TL | A04_valid_transitions | Planned | Planned | Planned | |
| A05 | Core | docs/spec/core/action_core.md | Invalid transitions rejected | TL | A05_invalid_transitions_rejected | Planned | Planned | Planned | |
| A06 | Core | docs/spec/core/action_core.md | Status updates monotonic | TL | A06_status_updates_monotonic | Planned | Planned | Planned | |
| A07 | Core | docs/spec/core/action_core.md | Event ordering determined by sequence, not wall-clock time | TL | A07_ordering_time_independent | Planned | Planned | Planned | |
| A08 | Core | docs/spec/core/action_core.md | Cancellation intent observable | TL | A08_cancel_intent_observable | Planned | Planned | Planned | |
| A09 | Core | docs/spec/core/action_core.md | Cancel resolution policy-defined | TL | A09_cancel_resolution_policy | Planned | Planned | Planned | |
| A10 | Core | docs/spec/core/action_core.md | Result visible after terminal | TL | A10_result_visible_after_terminal | Planned | Planned | Planned | |
| A11 | Core | docs/spec/core/action_core.md | Single terminal result | TL | A11_single_terminal_result | Planned | Planned | Planned | |
| A12 | Policy | docs/spec/action.md | Supersession via CANCELED | COM | A12_supersession_cancelled | Planned | Planned | Planned | Unvalidated. Likely ORACLE_B only; policy-layer behaviour |
| A13 | Policy | docs/spec/action.md | Result retention non-zero | TL | A13_result_retention | Planned | Planned | Planned | |
| A14 | Tooling | docs/spec/action.md | Tooling compatibility | TL | A14_tooling_compatibility | Planned | Planned | Planned | |
| L01 | Core | docs/spec/core/lifecycle_core.md | Only defined transitions | TL | L01_defined_transitions | Planned | Planned | Planned | |
| L02 | Core | docs/spec/core/lifecycle_core.md | Invalid transitions rejected | TL | L02_invalid_transitions_rejected | Planned | Planned | Planned | |
| L03 | Core | docs/spec/core/lifecycle_core.md | Intermediate states hidden | TL | L03_intermediate_states_hidden | Planned | Planned | Planned | |
| L04 | Core | docs/spec/core/lifecycle_core.md | Single active transition | TL | L04_single_active_transition | Planned | Planned | Planned | |
| L05 | Core | docs/spec/core/lifecycle_core.md | Concurrent transition rejected | TL | L05_concurrent_transition_rejected | Planned | Planned | Planned | |
| L06 | Core | docs/spec/core/lifecycle_core.md | ErrorProcessing deterministic | TL | L06_error_processing_deterministic | Planned | Planned | Planned | |
| L07 | Core | docs/spec/lifecycle.md | GetState reports primary only | TL | L07_get_state_primary | Planned | Planned | Planned | |
| L08 | Core | docs/spec/lifecycle.md | Rejection observable | TL | L08_rejection_observable | Planned | Planned | Planned | |
| L09 | Core | docs/spec/lifecycle.md | TransitionEvent emitted | TL | L09_transition_event_emitted | Planned | Planned | Planned | |
| L10 | Core | docs/spec/lifecycle.md | Shutdown deterministic | TL | L10_shutdown_deterministic | Planned | Planned | Planned | |
| G01 | Core (support) | docs/spec/lifecycle.md | Inactive suppresses pubs | TL | G01_inactive_suppresses_pubs | Planned | Planned | Planned | Gate rule |
| P01 | Core | docs/spec/core/parameter_core.md | No fabricated parameters | TL | P01_no_fabricated_params | Planned | Planned | Planned | |
| P02 | Core | docs/spec/core/parameter_core.md | Dynamic typing rules | TL | P02_dynamic_typing_rules | Planned | Planned | Planned | |
| P03 | Core | docs/spec/core/parameter_core.md | Redeclaration fails | TL | P03_redeclaration_fails | Planned | Planned | Planned | |
| P04 | Core | docs/spec/core/parameter_core.md | No partial application | TL | P04_no_partial_application | Planned | Planned | Planned | |
| P05 | Core | docs/spec/core/parameter_core.md | Read-only enforcement | TL | P05_readonly_enforcement | Planned | Planned | Planned | |
| P06 | Core | docs/spec/core/parameter_core.md | Unknown params rejected | TL | P06_unknown_params_rejected | Planned | Planned | Planned | |
| P07 | Core | docs/spec/core/parameter_core.md | Change record ordering | TL | P07_change_record_ordering | Planned | Planned | Planned | |
| P08 | Core | docs/spec/parameters.md | Unknown types NOT_SET | TL | P08_unknown_types_not_set | Planned | Planned | Planned | |
| P09 | Core | docs/spec/parameters.md | Atomic updates (external) | TL | P09_atomic_updates | Planned | Planned | Planned | |
| P10 | Core | docs/spec/parameters.md | Rejected updates silent | TL | P10_rejected_updates_silent | Planned | Planned | Planned | |
| P11 | Core | docs/spec/parameters.md | Events describe changes | TL | P11_events_describe_changes | Planned | Planned | Planned | |
| P12 | Core | docs/spec/parameters.md | Describe unknown consistent | TL | P12_describe_unknown_consistent | Planned | Planned | Planned | |
| P13 | Tooling | docs/spec/parameters.md | List complete and stable | TL | P13_list_complete_stable | Planned | Planned | Planned | |
| P14 | Policy | docs/spec/parameters.md | Deletion reported via events | TL | P14_deletion_reported_events | Planned | Planned | Planned | |
| S01 | System | docs/spec/system/composition.md | Load node observable | NONE | S01_load_node_observable | Planned | Planned | Planned | |
| S02 | System | docs/spec/system/composition.md | Load failure clean | NONE | S02_load_failure_clean | Planned | Planned | Planned | |
| S03 | System | docs/spec/system/composition.md | Unload node destroyed | NONE | S03_unload_node_destroyed | Planned | Planned | Planned | |
| S04 | System | docs/spec/system/composition.md | Unique node IDs | NONE | S04_unique_node_ids | Planned | Planned | Planned | |
| S05 | System | docs/spec/system/composition.md | Concurrent load deterministic | NONE | S05_concurrent_load_deterministic | Planned | Planned | Planned | |
| S06 | System | docs/spec/system/composition.md | Remaps apply at load | NONE | S06_remaps_apply_load | Planned | Planned | Planned | |
| S07 | System | docs/spec/system/executor.md | Explicit ownership | NONE | S07_explicit_ownership | Planned | Planned | Planned | |
| S08 | System | docs/spec/system/executor.md | Bounded spin modes | NONE | S08_bounded_spin_modes | Planned | Planned | Planned | |
| S09 | System | docs/spec/system/executor.md | Shutdown bounded | NONE | S09_shutdown_bounded | Planned | Planned | Planned | |
| S10 | System | docs/spec/system/executor.md | Callback failure visible | NONE | S10_callback_failure_visible | Planned | Planned | Planned | |
| S11 | System | docs/spec/system/executor.md | Lifecycle-Executor safety | NONE | S11_lifecycle_executor_safety | Planned | Planned | Planned | |
| S12 | System | docs/spec/system/ecosystem.md | Lifecycle orchestration | COM | S12_lifecycle_orchestration | Planned | Planned | Planned | |
| S13 | System | docs/spec/system/ecosystem.md | Liveness detection | COM | S13_liveness_detection | Planned | Planned | Planned | |
| S14 | System | docs/spec/system/ecosystem.md | Tooling interoperability | COM | S14_tooling_interoperability | Planned | Planned | Planned | |
| S15 | System | docs/spec/system/system_contract.md | Explicit ownership/lifetime | NONE | S15_explicit_ownership_lifetime | Planned | Planned | Planned | |
| S16 | System | docs/spec/system/system_contract.md | Deterministic bringup/teardown | NONE | S16_deterministic_bringup | Planned | Planned | Planned | |
| S17 | System | docs/spec/system/system_contract.md | No hidden execution | NONE | S17_no_hidden_execution | Planned | Planned | Planned | |
| S18 | System | docs/spec/system/system_contract.md | Observable failure | NONE | S18_observable_failure | Planned | Planned | Planned | |
| S19 | System | docs/spec/system/system_contract.md | Graph hygiene | NONE | S19_graph_hygiene | Planned | Planned | Planned | |
