## Quick TODO (phased, one-commit-per-phase)

* [x] Phase 1: Initial repository structure

* [x] Phase 2: Philosophy document (`docs/philosophy.md`)

* [x] Phase 3: Intent document (`docs/intent.md`)

* [x] Phase 4: Foundational README (`README.md`)

* [x] Phase 5: Spec boundary review (workbench)

  * `docs/workbench/spec_inventory.md`
  * commit: `docs: add spec inventory and boundary classification`

* [x] Phase 6: Import specs + provenance scaffolding

  * Move specs into `docs/spec/**`
  * Add provenance blocks to all specs
  * Add `docs/provenance/missing_semantics.md`
  * Add `docs/provenance/oracle_plan.md`
  * Add evidence surfaced at repo root (`evidence/**`)
  * commit: `docs: import semantic specs and system contracts`
  * commit: `docs: surface evidence artifacts at repo root`

* [x] Phase 7: Rust reference engine + contract tests

  * Import `ros2_semantics_core` into `reference/**`
  * Action / lifecycle / parameter cores implemented
  * All contract tests passing
  * commit: `ref: import rust reference semantics core + contract tests`

* [x] Phase 8a: Oracle harness baseline (H00)

  * `harness/**`
  * Stub backend (deterministic, non-ROS)
  * Core runner (judge + reporter)
  * Minimal trace vocabulary (v0.1-min)
  * H00 smoke scenario passes end-to-end
  * commit: `harness: add H00 stub backend + core runner`

* [ ] Phase 8b: ROS backend adapter (rclcpp)

  * Thin rclcpp adapter (no assertions, no policy)
  * commit: `harness: add rclcpp backend adapter`

* [ ] Phase 8c: Dockerized stacks (Jazzy / Nav2)

  * Containerized backends
  * commit: `harness: add dockerized production backends`

* [ ] Phase 9 (optional): rclpy comparison

  * commit: TBD
