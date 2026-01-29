## Quick TODO (phased, one-commit-per-phase)

- [x] Phase 1: Initial repository structure
- [x] Phase 2: Philosophy document (`docs/philosophy.md`)
- [x] Phase 3: Intent document (`docs/intent.md`)
- [x] Phase 4: Foundational README (`README.md`)

- [x] Phase 5: Spec boundary review (workbench)
  - `docs/workbench/spec_inventory.md`
  - commit: `docs: add spec inventory and boundary classification`

- [x] Phase 6: Import specs + provenance scaffolding
  - Move specs into `docs/spec/**`
  - Add provenance blocks to all specs
  - Add `docs/provenance/missing_semantics.md`
  - Add `docs/provenance/oracle_plan.md`
  - Add evidence surfaced at repo root (`evidence/**`)
  - commit: `docs: import semantic specs and system contracts`
  - commit: `docs: surface evidence artifacts at repo root`

- [ ] Phase 7: Move Rust reference engine + contract tests into `reference/**` + traceability links
  - commit: `ref: import rust reference semantics core + contract tests`

- [ ] Phase 8: Docker oracle harness (Jazzy + rclcpp + Nav2) in `harness/**`
  - commit: `harness: add Jazzy+rclcpp+Nav2 oracle runner`

- [ ] Phase 9 (optional): rclpy comparison (defer)
  - commit: TBD
