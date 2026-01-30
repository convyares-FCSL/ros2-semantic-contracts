# Semantic Traceability Matrix

This file maps each spec item to:
- provenance (top-level / community / none)
- oracle coverage (scenario IDs)
- production-stack traceability (where it is exercised)

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

| Spec ID | Spec doc | Clause / invariant | Provenance | Oracle scenario(s) | ORACLE_A | ORACLE_B | PROD | Notes |
|---|---|---|---|---|---|---|---|---|
| A1 | docs/spec/core/action_core.md | No ghosts after rejection | TL | A1_no_ghosts_after_rejection | Implemented (stub/ros adapter) | Planned | Planned | Tie to rclcpp action acceptance semantics |
