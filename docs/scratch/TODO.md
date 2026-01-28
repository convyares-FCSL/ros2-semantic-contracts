## Quick TODO (phased, one-commit-per-phase)

- [x] Phase 1: Initial repository structure  
- [x] Phase 2: Philosophy document (`docs/philosophy.md`)  
- [x] Phase 3: Intent document (`docs/intent.md`)  
- [x] Phase 4: Foundational README (`README.md`)  
* [ ] **Phase 5:** `docs/provenance/spec_inventory.md` (core vs adapter classification) — commit `docs: add spec inventory and boundary classification`
* [ ] **Phase 6:** Move core specs into `docs/spec/**` + add Provenance sections + `docs/provenance/missing_semantics.md` — commit `docs: import core semantic specs + provenance scaffolding`
* [ ] **Phase 7:** Move Rust reference engine + contract tests into `reference/**` + traceability links — commit `ref: import rust reference semantics core + contract tests`
* [ ] **Phase 8:** Docker oracle harness (Jazzy + rclcpp + Nav2) in `harness/**` — commit `harness: add Jazzy+rclcpp+Nav2 oracle runner`
* [ ] **Phase 9 (optional):** rclpy comparison (defer) — commit TBD


## Next

- [ ] Phase 5: Spec boundary review  
  - Produce `docs/provenance/spec_inventory.md`
  - Classify existing docs as:
    - CORE SPEC
    - CORE CONTRACT
    - ADAPTER SPEC
    - PARITY DOC
    - METHODOLOGY
    - OTHER
  - Decide what moves into this repo vs stays in adapter repos

## Upcoming

- [ ] Phase 6: Import core specs + provenance scaffolding  
- [ ] Phase 7: Import Rust reference semantics core + contract tests  
- [ ] Phase 8: Jazzy + rclcpp + Nav2 oracle harness  
- [ ] Phase 9: Optional rclpy comparison (deferred)





