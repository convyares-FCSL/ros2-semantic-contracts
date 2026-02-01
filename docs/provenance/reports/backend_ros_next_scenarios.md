# Backend ROS Next Scenarios — 2026-02-01

## A) Selection Criteria

Scenarios selected for immediate implementation on `backend_ros` must satisfy:

1. **Runnable without docker/Nav2** — local ROS 2 Jazzy only
2. **Cross-family coverage** — at least one from A/L/P where feasible
3. **Minimal new infrastructure** — reuse existing action/param/lifecycle harness nodes
4. **Measurable with existing instrumentation** — graph introspection, service calls, trace events

---

## B) Recommended Implementation Set (10 scenarios)

### Actions (5 scenarios)

| ID | File | Leverage | Minimal Ops/Expects | Dependencies |
|----|------|----------|---------------------|--------------|
| **A03** | scenarios_A.json | Unknown state not persisting — validates goal cleanup | `send_goal(unknown)`, wait, `list_goals` | Action server with unknown rejection |
| **A04** | scenarios_A.json | Valid transitions only — core state machine | `send_goal`, observe status changes | Existing action rig |
| **A05** | scenarios_A.json | Invalid transitions rejected — negative test | Attempt invalid transition | Existing action rig |
| **A08** | scenarios_A.json | Cancel intent observable — cancel flow | `send_goal`, `request_cancel`, observe cancel_accepted | Existing action rig |
| **A15** | scenarios_A.json | No Goal ID reuse — uniqueness constraint | Two goals with same UUID | Existing action rig |

### Parameters (4 scenarios)

| ID | File | Leverage | Minimal Ops/Expects | Dependencies |
|----|------|----------|---------------------|--------------|
| **P03** | scenarios_P.json | Redeclaration fails — prevents double-declare | `declare_param`, `declare_param` again | Parameter node |
| **P04** | scenarios_P.json | No partial application — atomic batch | `set_params([valid, invalid])`, verify neither applied | Parameter node |
| **P08** | scenarios_P.json | Unknown types NOT_SET — type safety | `describe_param(unknown)`, check type=NOT_SET | Parameter node (already works) |
| **P11** | scenarios_P.json | Events describe changes — param events | `set_param`, observe ParameterEvent | Parameter node + event subscription |

### Lifecycle (1 scenario)

| ID | File | Leverage | Minimal Ops/Expects | Dependencies |
|----|------|----------|---------------------|--------------|
| **L02** | scenarios_L.json | Invalid transitions rejected — lifecycle negative | Call `activate` from `unconfigured` | Lifecycle node |

---

## C) PR Breakdown Plan

| Branch | Scenarios | Rationale |
|--------|-----------|-----------|
| `feat/backend-ros-action-a03-a05` | A03, A04, A05 | Core action state machine validation |
| `feat/backend-ros-action-a08-a15` | A08, A15 | Cancel flow + uniqueness constraint |
| `feat/backend-ros-params-p03-p04` | P03, P04 | Declaration + atomicity |
| `feat/backend-ros-params-p08-p11` | P08, P11 | Type safety + events |
| `feat/backend-ros-lifecycle-l02` | L02 | Unlock lifecycle family |

---

## D) NOT Recommended Yet

| Family | Reason |
|--------|--------|
| G (Gating) | Requires pub/sub + lifecycle coordination |
| S (System) | Requires composition container infrastructure |
| A16/L13/P15 | Interface introspection — graph discovery timing risks |

---

## E) First Implementation Target

**Recommended: P08** (Unknown types NOT_SET)

- **Simplest:** Single `describe_param` call, single assertion
- **Already works:** describe operation exists via P12
- **Minimal risk:** No new backend hooks needed
- **High confidence:** Deterministic behavior

**Alternative: A03** if action-first preference

---

**Report generated:** 2026-02-01T21:08:48Z
