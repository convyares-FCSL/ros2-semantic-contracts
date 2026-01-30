### What I opened + read (grounding on *this* zip)

I extracted `harness.zip` and read **all non-binary files** (99/110 files). The remaining 11 are binaries/objects/static libs from `harness/build/` (e.g., `.a`, `.o`, compiled executables), which don’t contain authoritative source content.

The **actual harness** in this zip is:

* `harness/contracts/backend_contract.md` — states the desired separation (core judges, backends observe, JSONL append-only).
* `harness/configs/scenarios*.json` + `scenarios.schema.json` — scenario language currently includes `op: "assert"` and an `assert` payload (this pushes test semantics into the backend execution stream).
* `harness/configs/trace.schema.json` — models the trace as a **single JSON object with an `events` array**, but the implementation actually writes **JSONL** (one JSON object per line). That’s a schema/format mismatch.
* `harness/src/oracle_core.cpp` — implements a **stub model + expectation checking** by emitting `"assertion"` events containing `"ok"` and by writing `"scenario_end.detail.ok"`. That makes *verdict-like truth* part of the trace.
* `harness/src/ros_backend.cpp` — also does backend-side expectation checking via `emit_assert(...)` and emits `"assertion"` events with `"ok"` (direct contract violation).
* `harness/src/audit_trace.cpp` — treats `"assertion.detail.ok"` and `"scenario_end.detail.ok"` as authoritative pass/fail and returns process exit based on them.

So: the **contract document says the right thing**, but the **codebase currently violates it in two ways**:

1. the ROS backend asserts and emits `ok` (backend-side verdicting),
2. “truth” is embedded into the trace (`assertion.ok`, `scenario_end.ok`) instead of being a core-owned evaluation result.

---

## Step-by-step plan to convert to the new baseline (with “archive and start empty” as the default move)

### Phase 0 — Freeze and archive (keep as forensic reference)

1. Move the entire current `harness/` to `harness_legacy/` **as-is**, including `build/` (so you can reproduce old behaviour if needed). *[REMOVED 2026-01-30: harness_legacy deleted]*
2. In `harness_legacy/`, add a short `README_LEGACY.md` stating: “This implementation violates backend_contract separation; retained for history only.” *[REMOVED 2026-01-30]*

### Phase 1 — Create a clean new root with only the authoritative contract + schemas

3. Create a new `harness/` empty baseline with:

   * `contracts/backend_contract.md` (ported forward; treat as binding)
   * `schemas/` folder (new, versioned)
   * `scenarios/` folder (new)
4. **Delete** the idea of `assert` as an op. Scenarios become:

   * `ops` executed by backend
   * `expects/checks` evaluated by core
     Backends ignore expects completely.

### Phase 2 — Normalize the trace format (JSONL, backend-owned)

5. Define `trace_event.schema.json` for **one JSONL event**, not an `events[]` wrapper.
6. Remove `assertion.detail.ok` and `scenario_end.detail.ok` from the backend trace vocabulary.

   * Backends may emit `diagnostic` events, but never `ok/pass/fail`.
   * `scenario_end` from backend becomes “I’m done running ops” (optionally with transport failure info), not semantic status.

### Phase 3 — Build the Rust core runner (the judge)

7. Implement Rust `core` that:

   * loads scenario JSON
   * validates scenario schema
   * runs a backend process (stub or ROS)
   * validates each JSONL event against the trace event schema
   * evaluates expects over the trace
   * outputs **report.json** (+ human summary), and sets the final process exit code based on evaluation (not backend exit)

8. Replace `audit_trace.cpp` with a Rust “reporter” inside core (or keep a separate `audit` binary, but still Rust, still core-owned).

### Phase 4 — Rebuild the backends as thin adapters

9. Re-implement `backend_stub` as a deterministic event emitter:

   * reads ops
   * emits observations
   * exits 0 if it ran, nonzero only for transport/runtime problems

10. Re-implement `backend_ros_rclcpp` as a pure adapter:

* reads ops
* calls rclcpp
* emits observations (goal accepted/rejected, status samples, cancel requests/responses, result, etc.)
* emits diagnostics if useful
* **no** assertion helpers, **no** “expected vs observed”, **no** `ok`

### Phase 5 — Migration mapping (to avoid rewrite churn)

11. Map existing event types you already emit (`goal_send_decision`, `status`, `feedback`, `result`, `cancel_*`) into the new JSONL event schema.
12. Map existing scenario examples (A1/A2 JSON) into the new scenario format:

* keep the same ops semantics
* move asserts/expectations into core-side expects

### Phase 6 — Prove the new baseline early (A01/A02)

13. Implement only enough ops + event types to judge A01/A02 (your traceability doc already calls these out).
14. Lock it in with CI using `backend_stub` first, then `backend_ros_rclcpp`.
