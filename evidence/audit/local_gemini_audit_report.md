# ROS 2 Semantic Contracts — Comprehensive Audit Report

**Date:** 2026-01-30  
**Baseline:** ROS 2 Jazzy + rclcpp  
**Auditor:** AI-Assisted Review  

---

## Executive Summary

**Overall Assessment: Strong Foundation with Critical Gaps**

The `ros2-semantic-contracts` repository represents a **well-conceived and structurally sound** approach to codifying ROS 2 behavioral semantics. The epistemological framework is rigorous, the three-layer architecture (Core/Global/System) is coherent, and the measurement-driven philosophy is compelling. However, the repository suffers from **significant implementation gaps** that undermine its credibility as a production-ready reference.

### Verdict Matrix

| Category | Grade | Notes |
|----------|-------|-------|
| **Architecture & Design** | A | Excellent layered approach, clear separation of concerns |
| **Documentation Quality** | A- | Professional, clear, well-structured |
| **Technical Correctness** | B+ | Specs align with upstream; some gaps |
| **Traceability** | B+ | After recent fixes, 100% spec-to-scenario binding |
| **Validation Coverage** | D | Only 5 of ~65 scenarios PASS; most are N/A |
| **Link Hygiene** | C | 2 of 4 upstream URLs are broken (404) |
| **Harness Readiness** | C+ | Structure sound, but sparse implementation |
| **Community Reception Risk** | Medium | Early stage; claims exceed evidence |

---

## Strengths

### 1. Epistemological Rigor
The foundational documents (`philosophy.md`, `methodology.md`, `intent.md`) establish a defensible framework:
- **Measurement-driven**: "This repository does not fix ROS 2. It measures it."
- **Clear authority ladder**: Upstream → Production Stack → Ecosystem Evidence
- **Explicit uncertainty handling**: UNVALIDATED tags, hypothesis promotion rules

### 2. Three-Layer Architecture
The Core/Global/System separation is conceptually clean:
- **Core**: Transport-agnostic semantic invariants (engine-level)
- **Global**: ROS-facing observable behavior (wire protocol)
- **System**: Interoperability constraints (tooling, orchestration)

### 3. Provenance Discipline
Every normative claim has a provenance block with:
- Official upstream references
- REP/RFC citations (when applicable)
- Community/ecosystem evidence
- BIC (Baseline Interoperability Constraint) justification

### 4. Traceability (Post-Fix)
After the reconciliation session:
- **100% of specs have scenario bindings**
- **Zero orphaned specs**
- **Zero unbound scenarios** (in document layer)

### 5. Normative Language Consistency
```
124 × MUST
 32 × MUST NOT
  1 × SHALL
```
Appropriate RFC 2119 keyword usage. `SHALL` appears only once (correctly used).

---

## Critical Issues

### ISSUE #1: Validation Coverage is Sparse (BLOCKING)

**Evidence:**
| Status | Count | Percentage |
|--------|-------|------------|
| PASS | 5 | ~7% |
| SKIP | 2 | ~3% |
| N/A | ~58 | ~90% |

**Impact:** The repository claims to validate semantic contracts, but 90% of scenarios have no implementation. This undermines the core premise.

**Recommendation:** 
1. Prioritize A-series (Actions) and L-series (Lifecycle) validation
2. Consider marking unvalidated specs as `DRAFT` in headers
3. Add explicit "Validation Roadmap" with milestones

---

### ISSUE #2: Broken Upstream Links (HIGH)

**Broken URLs:**
1. `https://design.ros2.org/articles/composition.html` → **404 Not Found**
2. `https://docs.ros.org/en/jazzy/Concepts/Basic-Concepts/About-Parameters.html` → **404 Not Found**

**Working URLs:**
1. `https://design.ros2.org/articles/actions.html` → ✓ Valid
2. `https://design.ros2.org/articles/node_lifecycle.html` → ✓ Valid

**Impact:** Broken provenance links damage credibility and prevent verification.

**Recommendation:**
1. Replace composition URL with valid reference (likely moved or renamed)
2. Replace parameters URL with `https://docs.ros.org/en/jazzy/Concepts/Parameters.html` or similar
3. Add automated link-checking to CI

---

### ISSUE #3: Scenario Matrix Formatting Error (MEDIUM)

**Location:** `docs/provenance/scenario/scenario_execution_matrix.md`, line 99

**Problem:** Malformed markdown table row:
```markdown
| **S19** | S19_graph_hygiene                  | N/A  | N/A  | N/A  | N/A   | Not yet validated                  || A16 | A16_action_interface_conformity      | N/A | N/A | N/A | N/A | New: Interface Conformity |
```

The newly added scenarios (A16, L13, P15, S20) are appended inline, breaking the table structure.

**Recommendation:** 
Insert proper row separators:
```markdown
| **S19** | S19_graph_hygiene                  | N/A  | N/A  | N/A  | N/A   | Not yet validated                  |
| **A16** | A16_action_interface_conformity   | N/A  | N/A  | N/A  | N/A   | New: Interface Conformity          |
```

---

### ISSUE #4: Missing A16 Scenario Definition (MEDIUM)

**Problem:** A16 was added to scenario matrix but the semantics file (`semantics_A.md`) may be incomplete.

**Verification needed:** Confirm `semantics_A.md` contains full A16 definition with:
- Name/Title
- Scenario description
- Preconditions
- Steps
- Expected outcome

---

## Traceability Audit

### Status: PASS (Post-Reconciliation)

| Metric | Before | After |
|--------|--------|-------|
| Orphaned Specs | 6 | 0 |
| Unbound Scenarios (in docs) | 4 | 0 |
| Invalid Scenario References | 0 | 0 |

### Spec-to-Scenario Coverage

| Layer | Specs | Bound | Unbound | Coverage |
|-------|-------|-------|---------|----------|
| Core | 15 | 15 | 0 | 100% |
| Global | 24 | 24 | 0 | 100% |
| System | 26 | 26 | 0 | 100% |

---

## Spec-vs-Upstream Comparison

### Actions (Core: `action_core.md`)

| Spec Claim | Upstream Reference | Alignment |
|------------|-------------------|-----------|
| Goal identity via UUID | [design.ros2.org/actions.html](https://design.ros2.org/articles/actions.html) → "Goal Identifiers" | ✓ Aligned |
| Terminal states immutable | Upstream "Goal States" section defines SUCCEEDED, ABORTED, CANCELED as terminal | ✓ Aligned |
| Canceling → {Canceled, Succeeded, Aborted} | Upstream: "Notify that canceling the goal completed successfully" implies Canceled; "succeed/abort" transitions exist | ✓ Aligned |

### Lifecycle (Core: `lifecycle_core.md`)

| Spec Claim | Upstream Reference | Alignment |
|------------|-------------------|-----------|
| 4 Primary States | [design.ros2.org/node_lifecycle.html](https://design.ros2.org/articles/node_lifecycle.html) defines Unconfigured, Inactive, Active, Finalized | ✓ Aligned |
| Transition atomicity | Upstream defines transition states (Configuring, Activating, etc.) | ✓ Aligned |
| ErrorProcessing resolution | Upstream: "the node will transition to ErrorProcessing" | ✓ Aligned |

### Parameters

**WARNING:** Cannot verify - upstream link broken.

| Spec Claim | Status |
|------------|--------|
| Case-sensitive names | ⚠️ Unverifiable (link 404) |
| Type enforcement | ⚠️ Unverifiable (link 404) |
| Atomic set operations | ⚠️ Unverifiable (link 404) |

---

## Harness Readiness Assessment

### Structure: Sound

```
harness/
├── backends/        # 9 items (stub, ROS adapters)
├── contracts/       # 3 items (binding contracts)
├── core/           # 7 items (runner, evaluator)
├── scenarios/      # 6 items (JSON bundles)
├── schemas/        # 2 items (JSON schemas)
└── scripts/        # 1 item (run_harness.sh)
```

### Implementation: Sparse

| Backend | Status |
|---------|--------|
| `stub` | Functional (H00, A01, A02, P03 PASS) |
| `ros` | Partial (A01, A02 PASS, most N/A) |
| `rclrs` | Not implemented |
| `PROD` | Not applicable |

### Gap Analysis (Harness JSON vs Matrix)

After recent updates:
- **Missing implementations:** A03-A14, L01-L12, P01-P14, S01-S19
- **Placeholder entries:** A15, A16, A17, L11-L13, P15-P16, S20 (added but empty)

---

## Repository Hygiene

### Good Practices
- [x] Apache 2.0 License ✓
- [x] CONTRIBUTING.md with clear acceptance criteria ✓
- [x] Structured docs/ hierarchy ✓
- [x] Provenance subdirectory with divergence records ✓
- [x] Scenario execution matrix ✓

### Missing/Recommended
- [ ] CI/CD pipeline definition (`.github/workflows/`)
- [ ] CHANGELOG.md
- [ ] Automated link checker
- [ ] Pre-commit hooks for schema validation
- [ ] Version tagging (no releases yet)

---

## Reception Analysis

### Target Audience: ROS 2 Ecosystem Developers

**Positive Reception Factors:**
1. Fills a genuine gap in ROS 2 documentation
2. Professional, rigorously structured
3. Non-confrontational tone ("measures, not fixes")
4. Clear contribution guidelines

**Negative Reception Factors:**
1. Sparse validation coverage contradicts "semantic contracts" positioning
2. Broken upstream links suggest incomplete maintenance
3. No upstream engagement evidence (no REPs filed, no ros-discourse posts)
4. Heavy reliance on Nav2 as evidence may alienate non-navigation users

### Recommended Actions Before Public Promotion

1. **Achieve 50%+ scenario validation** before claiming "semantic contracts"
2. **Fix all broken links** (credibility killer for documentation)
3. **Engage ros-discourse** with design philosophy post
4. **Consider ROS Enhancement Proposal (REP)** for semantic contract framework
5. **Add Nav2 maintainer acknowledgment** if citing their behavior as evidence

---

## Prioritized Action List

### P0 — Blocking (Fix Immediately)

| # | Issue | File(s) | Status |
|---|-------|---------|--------|
| 1 | ~~Fix broken composition.html link~~ | `docs/spec/system/composition.md` | ✅ **FIXED** |
| 2 | ~~Fix broken Parameters URL~~ | `docs/spec/core/parameter_core.md`, `global/parameters.md` | ✅ **FIXED** |
| 3 | ~~Fix scenario matrix formatting~~ | `scenario_execution_matrix.md` | ✅ **FIXED** |

### P1 — High Priority (Before Public Promotion)

| # | Issue | File(s) | Effort |
|---|-------|---------|--------|
| 4 | Implement A03-A14 scenarios | `harness/scenarios/scenarios_A.json`, `backends/ros/` | High |
| 5 | Implement L01-L10 scenarios | `harness/scenarios/scenarios_L.json`, `backends/ros/` | High |
| 6 | Add CI/CD with link checker | `.github/workflows/` | Medium |
| 7 | Add CHANGELOG.md | Root | Low |

### P2 — Medium Priority (Post-Launch)

| # | Issue | File(s) | Effort |
|---|-------|---------|--------|
| 8 | Complete P-series validation | `harness/scenarios/scenarios_P.json` | High |
| 9 | Complete S-series validation | `harness/scenarios/scenarios_S.json` | High |
| 10 | Draft ros-discourse announcement | N/A | Medium |
| 11 | Evaluate REP for semantic contracts | N/A | Medium |

---

## Conclusion

The `ros2-semantic-contracts` repository is **architecturally excellent** but **operationally incomplete**. The vision is compelling, the structure is professional, and the documentation is clear. However, the sparse validation coverage (90% N/A) creates a significant gap between claims and evidence.

**Bottom Line:** This repository is currently a **thesis statement**, not a **proof**. With dedicated effort on scenario implementation and link fixes, it could become a valuable contribution to the ROS 2 ecosystem.

**Recommended Next Step:** Focus on P0 items immediately, then systematically implement A-series and L-series scenarios to reach 50% validation coverage before any public promotion.
