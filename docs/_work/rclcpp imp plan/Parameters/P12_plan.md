● Unified P12 Implementation Plan Complete

  I've regenerated a single comprehensive plan with all corrections integrated.

  Document Location

  File:
  /tmp/claude-1000/-home-ecm-ai-workspace-projects-ros2-semantic-contracts/04da3fa1-e0ce-406c-8e43-a76fb19d12ef/scratchpad/P12_implementation_plan_v2.md

  ---
  What's Integrated

  ✅ Issue 1: describe_param Justification (Section 3)

  - Explicit rationale for why it's a new op vs. extending get_param
  - Maps to distinct rclcpp APIs: get_parameter() (value) vs. describe_parameters() (metadata)
  - Shows consistency with P08 philosophy: both normalize distinct ROS APIs as observation primitives
  - Rejected alternatives documented (extend get_param, mode flags)

  ✅ Issue 2: Subset Matching Fixed (Sections 2, 4)

  - Canonical subset defined: type, read_only, dynamic_typing (3 fields)
  - Exhaustive matching on these 3 fields enforces consistency
  - Limitation documented: description/constraints not validated (acceptable)
  - Expects pattern matches all 3 required fields with exact values
  - Explains WHY subset matching works: both events must match same pattern → consistency proven for canonical subset

  ✅ Issue 3: Matrix Update Sequenced Correctly (Section 6, Step 11)

  - Moved from step 9 → step 11 (second-to-last)
  - Explicitly conditional: "ONLY proceed if Steps 8-10 all passed"
  - Notes field documents observed behavior (from trace), not assumptions
  - Matrix update included in final commit (evidence-based)

  ✅ Issue 4: Dependencies Clarified (Section 7)

  - Scenario-level (runtime): NONE — P12 can run independently
  - Infrastructure-level (build-time): YES — requires new op, schema, dispatch, capabilities
  - Explicitly states: "tooling dependency, not scenario dependency"

  ---
  Plan Structure (11 Sections + 3 Appendices)

  1. What P12 Proves — SPEC_P06 mapping, canonical subset definition, falsifiable evidence
  2. Evidence Strategy — op_end.detail encoding, why subset matching works (with corrections)
  3. Required Ops — describe_param justification, implementation, exception handling
  4. Scenario Design — ops (unchanged), expects (rewritten), notes (updated)
  5. Schema/Dispatch/Capabilities — exact file changes for 5 files with line numbers
  6. Verification Checklist — 13-step ordered sequence (baseline check → commit → push)
  7. Dependencies — scenario-level (none) vs. infrastructure-level (yes)
  8. Risks and Mitigations — 4 risks analyzed with mitigations
  9. Success Criteria — 9 checkboxes (all must be true)
  10. File Change Summary — table with ~50 lines across 5 files
  11. Implementation Touch List — ordered execution table (step 0-13)

  Appendices:
  - A: SPEC_P06 full text
  - B: Alternative approaches (rejected with rationale)
  - C: ParameterDescriptor field reference

  ---
  Key Corrections Applied

  Canonical Subset Definition (Section 1)

  Consistency validated for:
  - type (uint8)
  - read_only (bool)
  - dynamic_typing (bool)

  NOT validated:
  - description (string) — documentation metadata
  - additional_constraints (string)
  - range fields — complex arrays

  Rationale: Type-system fields define semantic behavior (critical).
            Description fields are human-readable metadata (secondary).

  Expects Pattern (Section 4)

  {
    "type": "op_end",
    "detail": {
      "op": "describe_param",
      "name": "param_does_not_exist",
      "descriptor": {
        "type": 0,              // ← MUST match exactly
        "read_only": false,     // ← MUST match exactly
        "dynamic_typing": false // ← MUST match exactly
      }
    }
  }
  Both op_end events must match this → if either differs, test FAILS.

  Verification Sequence (Section 6)

  Step 8:  Run P12 single scenario → inspect trace → verify descriptor
  Step 9:  Run determinism (5x) → verify byte-identical
  Step 10: Run full suite → verify no regressions
  Step 11: **CONDITIONAL:** Update matrix (only if 8-10 pass)
  Step 12: Commit (includes matrix)
  Step 13: Push

  ---
  What's Different from v1
  ┌──────────────────────────────┬──────────────────────────────┬───────────────────────────────────────────────────────┐
  │            Aspect            │        Original (v1)         │                     Unified (v2)                      │
  ├──────────────────────────────┼──────────────────────────────┼───────────────────────────────────────────────────────┤
  │ describe_param justification │ Missing                      │ Section 3 added (1 page)                              │
  ├──────────────────────────────┼──────────────────────────────┼───────────────────────────────────────────────────────┤
  │ Subset matching explanation  │ Claimed "proves consistency" │ Fixed: Defines canonical subset, documents limitation │
  ├──────────────────────────────┼──────────────────────────────┼───────────────────────────────────────────────────────┤
  │ Matrix update timing         │ Step 9 (premature)           │ Step 11 (conditional, last)                           │
  ├──────────────────────────────┼──────────────────────────────┼───────────────────────────────────────────────────────┤
  │ Dependencies                 │ "No dependencies"            │ Clarified: None (scenario), Yes (infrastructure)      │
  ├──────────────────────────────┼──────────────────────────────┼───────────────────────────────────────────────────────┤
  │ Expects pattern              │ Subset match (implicit)      │ Explicit: 3 required fields exhaustive                │
  ├──────────────────────────────┼──────────────────────────────┼───────────────────────────────────────────────────────┤
  │ Plan structure               │ Original sections            │ Integrated corrections inline                         │
  └──────────────────────────────┴──────────────────────────────┴───────────────────────────────────────────────────────┘
  ---
  Ready for Execution?

  The plan is now:
  - ✅ Justified (describe_param as separate op)
  - ✅ Correct (subset matching enforces consistency for canonical fields)
  - ✅ Sequenced (matrix update conditional, last step)
  - ✅ Precise (dependencies clarified)
  - ✅ Complete (13-step checklist, 5 files, ~50 lines, code snippets)
  - ✅ Actionable (file paths, line numbers, exact commands)

  Total implementation time: ~30-45 minutes (assuming no unexpected rclcpp behavior).

  Next step: Execute Step 0 (baseline check) to ensure H00, A01, A02, P03-P08 still PASS before starting P12 work.

✻ Brewed for 4m 32s

❯ No create a updated implementation plan, no history just the plan

● Write(/tmp/claude-1000/-home-ecm-ai-workspace-projects-ros2-semantic-contracts/04da3fa1-e0ce-406c-8e43-a76fb19d12ef/scratchpad/P12_implementation_plan_fin
       al.md)
  ⎿  Wrote 600 lines to ../../../../../tmp/claude-1000/-home-ecm-ai-workspace-projects-ros2-semantic-contracts/04da3fa1-e0ce-406c-8e43-a76fb19d12ef/scra
     tchpad/P12_implementation_plan_final.md
      1 # P12 — Describe Unknown Consistent: Implementation Plan
      2
      3 **Scenario:** P12_describe_unknown_consistent
      4 **SPEC:** SPEC_P06 (Global) — Consistent Description
      5 **Status:** Ready for Implementation
      6
      7 ---
      8
      9 ## 1. Objective
     … +590 lines (ctrl+o to expand)

● Clean Implementation Plan Ready

  Location:
  /tmp/claude-1000/-home-ecm-ai-workspace-projects-ros2-semantic-contracts/04da3fa1-e0ce-406c-8e43-a76fb19d12ef/scratchpad/P12_implementation_plan_final.md

  ---
  Structure (11 Sections)

  1. Objective — What P12 proves, consistency definition
  2. Why describe_param is a New Op — Justification
  3. Implementation Overview — 5 files, ~50 lines
  4. Scenario Design — Ops (unchanged), expects (rewritten), notes
  5. Code Changes — Exact diffs for all 5 files
  6. Implementation Sequence — 13 ordered steps (Step 0 → Step 13)
  7. Success Criteria — 9 checkboxes
  8. Dependencies — None (scenario), Yes (infrastructure)
  9. Risks — 3 risks with mitigations
  10. Expected Descriptor Structure — rclcpp Jazzy baseline
  11. Trace Event Schema Reference — Approved types only

  ---
  Key Points

  Consistency Definition

  - Validated: type, read_only, dynamic_typing (3 fields)
  - Not validated: description, constraints (documented limitation)
  - Proof: Both op_end events must match identical pattern

  Expected Descriptor (Unknown Param)

  {
    "type": 0,
    "read_only": false,
    "dynamic_typing": false
  }

  Implementation Sequence

  0. Baseline check (all prior scenarios PASS)
  1. Update schema enum
  2. Validate bundles
  3. Implement exec_describe_param (params.cpp)
  4. Update backend dispatch (backend.cpp)
  5. Rewrite P12 expects (scenarios_P.json)
  6. Validate bundles again
  7. Build
  8. Run P12 single scenario → inspect trace
  9. Determinism check (5 runs)
  10. Full suite (regression check)
  11. Update matrix (CONDITIONAL — only if 8-10 pass)
  12. Commit
  13. Push

  Files Modified

  - scenario.schema.json (+1 line)
  - backend.cpp (+3 lines)
  - params.cpp (+40 lines)
  - scenarios_P.json (~5 lines net)
  - scenario_execution_matrix.md (+1 line, conditional)

  ---
  No History, Just Action

  The plan contains:
  - ✅ Exact code snippets (copy-paste ready)
  - ✅ Exact file paths and line numbers
  - ✅ Exact bash commands
  - ✅ Clear success criteria for each step
  - ✅ Risk mitigations
  - ✅ Expected outcomes

  No references to:
  - ❌ v1/v2
  - ❌ Corrections
  - ❌ What changed
  - ❌ Issues

  Just a clean, professional implementation plan ready for execution.
