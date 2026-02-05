# Backend ROS Suitability Report — 2026-02-01

## Executive Summary

Of 68 total scenarios across 6 families (H/A/L/P/G/S), **only 5 have complete harness implementations** (ops + expects defined). The rest are skeleton stubs awaiting implementation.

**RUNNABLE on backend_ros now:** H00, A01, A02, P06, P12
**Already executed:** H01, A01, A02 (per execution matrix)
**Blockers:** 63 scenarios have empty ops/expects (stubs)

The `backend_ros` is a Rust implementation with action-focused ops (`send_goal`, `complete_terminal`, etc.). Lifecycle and system scenarios require additional op implementations.

---

## Scenario Suitability Table

| ID | Family | Runnable? | Reason | Harness Impl? | Evidence Path |
|----|--------|-----------|--------|---------------|---------------|
| **H00** | Harness | ✅ RUNNABLE | Complete ops/expects, harness self-test | ✓ Full | `harness/scenarios/scenarios_H.json` |
| **A01** | Action | ✅ RUNNABLE | Complete ops/expects, basic action flow | ✓ Full | `harness/scenarios/scenarios_A.json` |
| **A02** | Action | ✅ RUNNABLE | Complete ops/expects, terminal immutability | ✓ Full | `harness/scenarios/scenarios_A.json` |
| A03 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A04 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A05 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A06 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A07 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A08 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A09 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A10 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A11 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub) | Stub | — |
| A12 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub, Policy layer) | Stub | — |
| A13 | Action | ❌ NOT RUNNABLE | Empty ops/expects (stub, Policy layer) | Stub | — |
| A14 | Action | ⚠️ UNKNOWN | Interface introspection type, needs tooling | Planned | — |
| A15 | Action | ❌ NOT RUNNABLE | Status: planned | Planned | — |
| A16 | Action | ⚠️ RUNNABLE? | Interface introspection, implemented flag | Implemented | — |
| A17 | Action | ❌ NOT RUNNABLE | Status: planned (System layer) | Planned | — |
| **L01-L10** | Lifecycle | ❌ NOT RUNNABLE | All stubs (empty ops/expects) | Stub | — |
| L11 | Lifecycle | ❌ NOT RUNNABLE | Status: planned | Planned | — |
| L12 | Lifecycle | ❌ NOT RUNNABLE | Status: planned | Planned | — |
| L13 | Lifecycle | ⚠️ RUNNABLE? | Interface introspection, implemented | Implemented | — |
| **P01-P05** | Parameter | ❌ NOT RUNNABLE | Stubs (empty ops/expects) | Stub | — |
| **P06** | Parameter | ✅ RUNNABLE | Complete ops/expects, param rejection | ✓ Full | `harness/scenarios/scenarios_P.json` |
| P07-P11 | Parameter | ❌ NOT RUNNABLE | Stubs (empty ops/expects) | Stub | — |
| **P12** | Parameter | ✅ RUNNABLE | Complete ops/expects, describe consistency | ✓ Full | `harness/scenarios/scenarios_P.json` |
| P13-P14 | Parameter | ❌ NOT RUNNABLE | Stubs | Stub | — |
| P15 | Parameter | ⚠️ RUNNABLE? | Interface introspection, implemented | Implemented | — |
| P16 | Parameter | ❌ NOT RUNNABLE | Status: planned | Planned | — |
| **G01** | Gating | ❌ NOT RUNNABLE | Stub (empty ops/expects) | Stub | — |
| **S01-S19** | System | ❌ NOT RUNNABLE | All stubs (require composition/executor infra) | Stub | — |
| S20 | System | ⚠️ RUNNABLE? | Interface introspection, implemented | Implemented | — |

---

## Capability Labels (Inferred)

| Capability | Scenarios Using | backend_ros Support |
|------------|-----------------|---------------------|
| `actions.basic` | A01 | ✅ Supported |
| `actions.terminal` | A02 | ✅ Supported |
| `actions.cancel` | A08, A09 | 🔧 Needs impl |
| `lifecycle.services` | L01-L13 | 🔧 Needs impl |
| `lifecycle.events` | L09 | 🔧 Needs impl |
| `params.set` | P06 | ✅ Supported |
| `params.describe` | P12 | ✅ Supported |
| `params.atomic` | P04, P09 | 🔧 Needs impl |
| `graph.observe` | A16, L13, P15, S20 | ⚠️ Interface only |
| `composition.container` | S01-S07 | 🔧 Needs infra |
| `executor.spin` | S08-S11 | 🔧 Needs infra |
| `harness.smoke` | H00 | ✅ Supported |

---

## Blockers (Grouped)

### Missing Harness Implementation (ops/expects empty)

**63 scenarios** have skeleton stubs. Key groups:
- A03-A15 (13 scenarios)
- L01-L12 (12 scenarios)
- P01-P05, P07-P11, P13-P14, P16 (13 scenarios)
- G01 (1 scenario)
- S01-S19 (19 scenarios)

### Requires Special Infrastructure

| Scenario Group | Requirement |
|----------------|-------------|
| S01-S07 | Composition container manager |
| S08-S11 | Executor introspection |
| S12-S19 | System orchestration / Nav2 |
| A17 | Client disconnect detection |

### Known Flakiness Risks

| Scenario | Risk |
|----------|------|
| A16, L13, P15, S20 | Graph discovery timing (interface introspection) |
| L09 | Transition event timing |
| S13 | Liveness detection timing windows |

---

## Concrete Next Action

**Implement ops/expects for L01 and L07** — These are foundational lifecycle scenarios (valid transitions, get_state) that require only lifecycle service calls, which rclrs supports. This will unlock the lifecycle family for ros_local execution.

---

**Report generated:** 2026-02-01T20:32:40Z
