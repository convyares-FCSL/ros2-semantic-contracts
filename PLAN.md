# Production Track Plan: Two-Track Strategy

> **Goal**: Incrementally grow `backend_prod` with rigorous quality, validate value through divergence detection, then expand to Python only when evidence justifies the investment.

---

## Overview

This plan defines a **two-track production strategy** for implementing ROS 2 semantic contract scenarios:

- **Track 1 (Production C++ / backend_prod)**: Implement scenarios in a sensible order that incrementally grows operations and features with rigorous quality standards
- **Track 2 (Python/rclpy)**: Begin only after a defined "switch gate" is met, and only for a small subset to validate divergence and value

**Baseline**: H00 + probe infrastructure is locked and merged via PR on `dev` branch.

---

## Track 1: Production C++ (backend_prod)

### First 10 Scenarios (Ranked Implementation Order)

The following scenarios are ranked to maximize incremental capability growth, minimize dependencies, and enable early divergence detection.

#### 1. **P08** — Unknown Types NOT_SET
- **Scenario ID**: `P08`
- **Family**: Parameters (Core)
- **New Ops/Capabilities**:
  - `describe_param` operation
  - Parameter descriptor response handling
  - Type validation (NOT_SET for unknown parameters)
- **Why Next**:
  - **Dependency**: Minimal - reuses existing parameter node infrastructure
  - **Value**: Simplest parameter scenario; validates type safety contract
  - **Risk**: Very low - deterministic behavior, already works in P12 context
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - returns `type: NOT_SET` for undeclared parameters
  - **rclcpp (Humble)**: PASS (expected, measure to confirm)
  - **rclpy**: Unknown - measure to detect divergence

---

#### 2. **P03** — Redeclaration Fails
- **Scenario ID**: `P03`
- **Family**: Parameters (Core)
- **New Ops/Capabilities**:
  - `declare_param` operation
  - Redeclaration detection and rejection
  - Error handling for duplicate declarations
- **Why Next**:
  - **Dependency**: Builds on P08 (parameter ops exist)
  - **Value**: Core invariant - prevents double-declaration bugs
  - **Risk**: Low - well-defined failure mode
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - throws/returns failure on redeclaration
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - measure for divergence

---

#### 3. **P04** — No Partial Application
- **Scenario ID**: `P04`
- **Family**: Parameters (Core)
- **New Ops/Capabilities**:
  - `set_params` batch operation
  - Atomic transaction validation
  - Rollback on partial failure
- **Why Next**:
  - **Dependency**: Requires P03 (declare) + P08 (type validation)
  - **Value**: Critical for safety - validates atomicity contract
  - **Risk**: Medium - requires careful transaction handling
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - all-or-nothing batch updates
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - likely divergence candidate

---

#### 4. **L02** — Invalid Transitions Rejected
- **Scenario ID**: `L02`
- **Family**: Lifecycle (Core)
- **New Ops/Capabilities**:
  - `change_state` operation
  - Lifecycle state machine validation
  - Invalid transition rejection
  - Service response error codes
- **Why Next**:
  - **Dependency**: Unlocks lifecycle family (first lifecycle scenario)
  - **Value**: Core state machine invariant - foundational for all lifecycle work
  - **Risk**: Low - negative test with well-defined rejection
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - rejects invalid transitions (e.g., unconfigured → active)
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - measure

---

#### 5. **P11** — Events Describe Changes
- **Scenario ID**: `P11`
- **Family**: Parameters (Global)
- **New Ops/Capabilities**:
  - `/parameter_events` topic subscription
  - Event observation and validation
  - Event content assertions (new value present)
- **Why Next**:
  - **Dependency**: Builds on P03/P04 (set operations exist)
  - **Value**: Validates observable contract - events describe state changes
  - **Risk**: Medium - requires event timing and content validation
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - events contain new values
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - event format may diverge

---

#### 6. **A03** — Unknown Goal Not Persistent
- **Scenario ID**: `A03`
- **Family**: Actions (Core)
- **New Ops/Capabilities**:
  - `send_goal(unknown)` mode (goal rejection)
  - Goal status array observation
  - Cleanup validation (goal not in status)
- **Why Next**:
  - **Dependency**: Extends H00 action infrastructure (send_goal exists)
  - **Value**: Core action invariant - validates goal lifecycle
  - **Risk**: Low - extends existing action rig
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - unknown goals not observable in status
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - measure

---

#### 7. **A04** — Valid Transitions Only
- **Scenario ID**: `A04`
- **Family**: Actions (Core)
- **New Ops/Capabilities**:
  - Action status observation over time
  - State transition sequence validation
  - State machine edge enforcement
- **Why Next**:
  - **Dependency**: Builds on A03 (action status observation)
  - **Value**: Core action state machine - foundational for all action work
  - **Risk**: Low - positive test of canonical transitions
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - follows canonical state machine
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - measure

---

#### 8. **A08** — Cancellation Intent Observable
- **Scenario ID**: `A08`
- **Family**: Actions (Core)
- **New Ops/Capabilities**:
  - `request_cancel` operation
  - Cancel service interaction
  - Canceling state observation
- **Why Next**:
  - **Dependency**: Requires A04 (state transitions work)
  - **Value**: Critical for production - validates cancel flow
  - **Risk**: Medium - requires cancel service + status coordination
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - cancel intent observable via status
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - likely divergence candidate

---

#### 9. **L05** — Concurrent Transition Rejected
- **Scenario ID**: `L05`
- **Family**: Lifecycle (Core)
- **New Ops/Capabilities**:
  - Concurrent `change_state` calls
  - Busy/rejection handling during active transition
  - State machine locking validation
- **Why Next**:
  - **Dependency**: Requires L02 (lifecycle ops exist)
  - **Value**: Critical for safety - prevents race conditions
  - **Risk**: Medium - requires timing/concurrency handling
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - rejects concurrent transitions
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - concurrency handling may diverge

---

#### 10. **A15** — No Goal ID Reuse
- **Scenario ID**: `A15`
- **Family**: Actions (Core)
- **New Ops/Capabilities**:
  - UUID uniqueness validation
  - Goal ID collision detection
  - Session-wide ID tracking
- **Why Next**:
  - **Dependency**: Requires A03/A04 (goal lifecycle works)
  - **Value**: Core invariant - prevents ID collision bugs
  - **Risk**: Low - deterministic uniqueness check
- **Expected Outcome**:
  - **rclcpp (Jazzy)**: PASS - rejects/handles duplicate UUIDs
  - **rclcpp (Humble)**: PASS (expected)
  - **rclpy**: Unknown - measure

---

### Summary: Track 1 Capability Growth

| Scenario | Family | New Capability Delta |
|----------|--------|---------------------|
| P08 | Params | Parameter describe, type validation |
| P03 | Params | Parameter declare, redeclaration detection |
| P04 | Params | Batch set, atomic transactions |
| L02 | Lifecycle | Lifecycle state machine, invalid transition rejection |
| P11 | Params | Parameter events subscription, event validation |
| A03 | Actions | Goal rejection, status cleanup validation |
| A04 | Actions | Action state transitions, sequence validation |
| A08 | Actions | Cancel flow, cancel service |
| L05 | Lifecycle | Concurrent transition handling, state locking |
| A15 | Actions | UUID uniqueness, ID collision detection |

**Incremental Growth Pattern**:
1. **Scenarios 1-3**: Parameter foundation (describe → declare → batch set)
2. **Scenario 4**: Lifecycle foundation (state machine)
3. **Scenario 5**: Parameter observability (events)
4. **Scenarios 6-8**: Action foundation (rejection → transitions → cancel)
5. **Scenario 9**: Lifecycle concurrency (safety)
6. **Scenario 10**: Action uniqueness (safety)

---

## Track 2: Python (rclpy) — Switch Gate

### Gate Criteria (Objective and Short)

Python implementation begins **only after** all of the following conditions are met:

1. **Stability Gate**: At least **8 of 10** Track 1 scenarios pass consistently (5/5 runs) in Docker on Jazzy `backend_prod`
2. **Capability Gate**: The following operations are stable in `backend_prod`:
   - Parameters: `declare_param`, `set_params`, `describe_param`, parameter events
   - Lifecycle: `change_state`, state validation, concurrent rejection
   - Actions: `send_goal`, `request_cancel`, status observation
3. **Evidence Validity Gate**: All passing scenarios produce:
   - Non-empty trace artifacts (`.jsonl` files)
   - Deterministic evidence (5/5 identical outcomes)
   - Valid trace schema (passes schema validation)

**Rationale**: Python work is only valuable if we have a stable baseline to compare against. Without rigorous C++ validation, divergence detection is meaningless.

---

### Python Scope (Minimal and Focused)

Once the switch gate is met, implement **only** the operations needed for **2-3 divergence probe scenarios**:

#### Recommended Python Probe Scenarios

| Scenario | Rationale |
|----------|-----------|
| **L05** (Concurrent Transition Rejected) | High divergence probability - Python GIL may affect concurrency handling |
| **P04** (No Partial Application) | High divergence probability - Python may have different transaction semantics |
| **A08** (Cancellation Intent Observable) | Medium divergence probability - Cancel flow timing may differ |

#### Python Implementation Constraints

- **Strict Fidelity Mode Only**: Use real `rclpy` APIs - no simulation, no cheating
- **Minimal Ops**: Implement only the operations required for the 3 probe scenarios:
  - `change_state` (for L05)
  - `set_params` (for P04)
  - `request_cancel` (for A08)
- **No Expansion**: Do NOT implement all 60 scenarios
- **No Distro Expansion**: Stay on Jazzy only until value is proven

**Deliverable**: A small `backend_rclpy` that can run 3 scenarios and emit comparable traces for divergence analysis.

---

## Explicit Non-Goals

To maintain focus and avoid scope creep:

1. **Do NOT implement all 60 scenarios** - Track 1 stops at 10, Track 2 at 3
2. **Do NOT polish probe backend into production** - `backend_ros` remains a probe, not production
3. **Do NOT expand distro matrix** - Stay on Jazzy until value is proven (no Humble/Rolling yet)
4. **Do NOT implement G (Gating) or S (System) families** - Requires pub/sub + composition infrastructure (future work)
5. **Do NOT implement interface introspection scenarios** (A16/L13/P15) - Graph discovery timing risks (future work)

---

## How We Validate Each New Scenario

For every new scenario added to `backend_prod`, follow this validation protocol:

### 1. Local Development Validation
```bash
# Build backend_prod
colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run H00 + new scenario locally
export ORACLE_BACKEND=prod
./harness/scripts/run_harness.sh harness/scenarios/scenarios_<FAMILY>.json /tmp/trace.jsonl /tmp/report.json

# Verify new scenario passes
cat /tmp/report.json | jq '.results[] | select(.scenario_id == "<NEW_SCENARIO_ID>")'
```

### 2. Docker Validation (5/5 Determinism)
```bash
# Build Docker image
docker compose build

# Run 5 times, collect evidence
for i in {1..5}; do
  docker compose up --abort-on-container-exit
  cp evidence/trace.jsonl evidence/trace_run_$i.jsonl
  cp evidence/report.json evidence/report_run_$i.jsonl
done

# Verify determinism (all 5 runs identical outcome)
sha256sum evidence/report_run_*.jsonl | awk '{print $1}' | sort -u | wc -l
# Expected: 1 (all hashes identical)
```

### 3. Probe Matrix Sanity Checks
```bash
# Run probe matrix to ensure no regressions
./harness/scripts/run_probe_matrix.sh

# Check for:
# - No new failures in existing scenarios
# - New scenario appears in matrix results
# - Evidence artifacts non-empty
```

### 4. Evidence Artifact Validation

For each scenario, verify:
- **Non-empty traces**: `trace.jsonl` file size > 0 bytes
- **Deterministic outcomes**: 5/5 runs produce identical PASS/FAIL verdict
- **Schema validity**: Trace events conform to schema (envelope + detail fields)
- **Event completeness**: All expected event types present (e.g., `goal_send`, `goal_response` for actions)

### 5. Acceptance Criteria

A scenario is considered **validated** when:
- ✅ Passes locally (1/1)
- ✅ Passes in Docker (5/5 deterministic)
- ✅ Probe matrix shows no regressions
- ✅ Evidence artifacts are non-empty and schema-valid
- ✅ Expected outcome matches actual outcome (PASS on Jazzy rclcpp)

---

## Success Metrics

### Track 1 Success (Production C++)
- **10 scenarios implemented** in `backend_prod`
- **8+ scenarios passing** consistently (5/5) on Jazzy
- **Evidence artifacts** for all scenarios (deterministic, schema-valid)
- **Zero regressions** in H00 baseline

### Track 2 Success (Python Divergence Probes)
- **Switch gate met** (8/10 stable, evidence valid)
- **3 scenarios implemented** in `backend_rclpy`
- **At least 1 divergence detected** (rclpy FAIL where rclcpp PASS)
- **Divergence record created** documenting the difference

### Overall Success
- **Value proven**: Divergence detection works, contracts are enforceable
- **Foundation stable**: Ready to expand to more scenarios/distros
- **Quality maintained**: No shortcuts, no simulation, rigorous validation

---

## Next Steps After This Plan

1. **Implement Track 1 scenarios** in order (P08 → P03 → P04 → L02 → P11 → A03 → A04 → A08 → L05 → A15)
2. **Validate each scenario** using the 5-step protocol above
3. **Monitor switch gate criteria** - track progress toward 8/10 stable
4. **Pause at gate** - do NOT start Python work until gate is met
5. **Implement Python probes** (L05, P04, A08) once gate is met
6. **Analyze divergence** - create divergence records for any failures
7. **Decide on expansion** - if value is proven, expand scope; otherwise, pivot

---

**Plan Status**: Draft  
**Branch**: `prod/track1-first10`  
**Baseline**: H00 + probe infra (merged to `dev`)  
**Author**: Antigravity Agent  
**Date**: 2026-02-03
