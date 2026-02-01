# Project Roadmap

* [x] **Phase 1: Structure & Philosophy**
    * [x] Philosophy (`docs/philosophy.md`)
    * [x] Intent (`docs/intent.md`)
    * [x] Spec Inventory (Migration complete)

* [x] **Phase 2: Specification Architecture**
    * [x] Define Three-Layer Architecture (Core / Global / System)
    * [x] Establish Epistemology (Oracle, Measurement, Baseline)
    * [x] **Core Contracts:** Actions, Lifecycle, Parameters
    * [x] **Global Specs:** Actions, Lifecycle, Parameters
    * [x] **System Contracts:** Composition, Executor, Ecosystem

* [x] **Phase 3: Oracle Definitions**
    * [x] Naming Rules & Traceability
    * [x] Measurement Standards ($T_{obs}$)
    * [x] Validation Plan (`oracle_plan.md`)
    * [x] Scenario Matrix (`scenario_execution_matrix.md`)

* [ ] **Phase 4: Harness Implementation (Next)**
    * [ ] **H00 Smoke Test:** Validate harness machinery
    * [ ] **Stub Backend:** Implement Reference Engine (Rust)
    * [ ] **ROS Backend:** Implement `rclcpp` Adapter
    * [ ] **Docker:** Containerize Jazzy+Nav2 Baseline

* [ ] **Phase 5: Validation & Divergence**
    * [ ] Run Oracle against Baseline
    * [ ] Populate `divergence_records/`
    * [ ] Promote `UNVALIDATED` specs to Normative