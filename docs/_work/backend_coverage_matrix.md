# Backend Coverage Matrix

This matrix records **observed and implemented semantic coverage** across backends.

It is **not inferred from backend self-reporting** and is **not derived automatically**
from traces. It is a maintained record of what has actually been implemented,
exercised, and judged by the oracle.

Backend capability events are **descriptive metadata only**.

---

## Legend

- PASS — scenario implemented and expectations satisfied on this backend
- FAIL — scenario implemented and expectations violated on this backend
- SKIP — scenario executed but not judgeable yet (no expectations)
- N/A — scenario not implemented for this backend

---

## Backends

- `stub` — minimal harness validation backend
- `ros` — ORACLE_A (Jazzy + rclcpp baseline witness)

---

## Coverage Matrix

| Spec | Scenario ID                    | stub | ros | Notes |
|-----:|--------------------------------|------|-----|-------|
| H00  | H00_harness_smoke               | PASS | PASS | Harness calibration |
| A01  | A01_unique_goal_identity        | PASS | PASS | Action goal identity |
| A02  | A02_terminal_immutability       | PASS | PASS | Terminal immutability |
| A03  | A03_unknown_not_persistent      | N/A  | SKIP | Not implemented yet |
| P06  | P06_unknown_params_rejected     | N/A  | N/A  | Next target |

---

## Rules

- “Unsupported” is represented as **N/A**, not PASS or FAIL.
- SKIP reflects lack of expectations or incomplete semantics, not success.
- This table is updated **only when code and scenarios change**.
- The oracle does not derive this table automatically.
