# Backend ROS Smoke Set — 2026-02-01

## Executive Summary

This smoke set contains **6 scenarios** that can be run on `backend_ros` (ros_local) today with full ops/expects definitions. It covers 4 of 6 semantic families and validates core harness functionality.

---

## Smoke Set

| # | ID | Family | Rationale |
|---|-----|--------|-----------|
| 1 | **H00** | Harness | Validates harness pipeline (load → run → trace → evaluate); must pass before any semantic tests |
| 2 | **A01** | Action | Demonstrates `actions.basic` — rejected goals produce no observable lifecycle events |
| 3 | **A02** | Action | Demonstrates `actions.terminal` — terminal state immutability is enforced |
| 4 | **P06** | Parameter | Demonstrates `params.set` — setting undeclared parameters is rejected |
| 5 | **P12** | Parameter | Demonstrates `params.describe` — describe calls are consistent for unknown params |
| 6 | **A16** | Action | Demonstrates `graph.observe` — action interface topology is discoverable (interface introspection) |

---

## Coverage Matrix

| Family | Covered? | Scenarios |
|--------|----------|-----------|
| H (Harness) | ✅ | H00 |
| A (Action) | ✅ | A01, A02, A16 |
| L (Lifecycle) | ❌ | None runnable (all stubs) |
| P (Parameter) | ✅ | P06, P12 |
| G (Gating) | ❌ | G01 is stub |
| S (System) | ❌ | All stubs / require infra |

---

## Execution Command

```bash
cd harness
ORACLE_BACKEND=ros ./scripts/run_harness.sh scenarios/scenarios_H.json
ORACLE_BACKEND=ros ./scripts/run_harness.sh scenarios/scenarios_A.json --filter A01,A02,A16
ORACLE_BACKEND=ros ./scripts/run_harness.sh scenarios/scenarios_P.json --filter P06,P12
```

*(Note: `--filter` syntax is aspirational; adjust per actual runner CLI)*

---

## Success Criteria

All 6 scenarios must produce:
1. Valid JSONL trace at `/tmp/oracle_trace.jsonl`
2. Verdict `PASS` in `/tmp/oracle_report.json`
3. No unexpected assertions or crashes

---

## Not Yet Runnable (Deferred to Stage 3)

| Family | Why Deferred |
|--------|--------------|
| L (Lifecycle) | All scenarios are stubs; need ops/expects |
| G (Gating) | G01 stub; requires lifecycle + pub/sub |
| S (System) | Requires composition/executor infrastructure |

---

## Follow-up Recommendation

**Priority:** Implement L01 and L07 to add lifecycle coverage to the smoke set. These scenarios require only lifecycle service calls (configure, get_state) which rclrs already supports.

---

**Report generated:** 2026-02-01T20:32:40Z
