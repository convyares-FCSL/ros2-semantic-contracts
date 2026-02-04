# Lessons Learned: Probe Matrix & Backend Development

## What Went Wrong / Areas for Improvement

*   **Custom Trace Events**: We initially defined custom event types (`transition_request`, `get_state_response`) for `rclpy` to debug probes. This violated the authoritative Oracle Schema, causing validation failures. **Fix**: Stick to `op_start` and `op_end` with `detail` fields for operation-specific data.
*   **Concurrency & Flakiness**: The `L05` probe relied on loose `time.sleep` calls to orchestrate concurrent transitions. This led to non-deterministic behavior where the "concurrent" transition sometimes happened sequentially. **Fix**: Use barriers or explicit state checks where possible, or use conservative sleeps with clear documentation (accepted for probes, but less ideal for strict contracts).
*   **Backend Parity Gaps**: `backend_prod` (C++) initially lacked parameter and lifecycle operations (`declare_param`, `change_state`), causing probes to fail or be unsupported. **Fix**: Implemented `NodeContext` to support both `rclcpp::Node` and `rclcpp_lifecycle::LifecycleNode`, and added full `LifecycleOps` implementation in C++ matching `rclpy` behavior. **Status: RESOLVED**.
*   **Strict Fidelity & Test Cheating**: Initially, the `rclpy` backend used custom locks and multi-threaded executors to "pass" probes L05 (Concurrency) and S11 (Starvation). This masked the reality that standard `SingleThreadedExecutor` ROS 2 nodes *do* starve and *do* queue concurrent requests. **Fix**: Refactored `rclpy` to `RCLPY_FIDELITY_MODE=strict`, removing artificial locks. Result: `rclpy` now faithfully fails L05/S11, validating that the probes correctly detect non-compliant behavior in standard constructs.
*   **Docker Rebuilds**: The initial script rebuilt containers for every cell, wasting significant time. **Fix**: "Build once, run many" pattern using `docker compose build` followed by `run --no-recreate` (or relying on image existence).
*   **Ambiguous "Unsupported" Status**: We lacked a clear way to distinguish "Test Failed" from "Feature Not Implemented" in the automated report. **Fix**: Explicitly mapped missing capabilities (like prod lifecycle) to `UNSUPPORTED` in the aggregation layer.

## Good Decisions

*   **Matrix Testing**: Running all combinations (humble/jazzy × prod/rclpy × probes) caught environment-specific and backend-specific issues early.
*   **Aggregation Script**: Moving result parsing to a dedicated Python script (`aggregate_results.py`) made it easier to generate human-readable summaries compared to complex Bash piping.
*   **Schema Enforcement**: Strictly validating the trace against the schema (even if it required rework) ensures long-term interoperability between backends and the oracle.

## Recommendations for Track-1 Backend Implementation

1.  **Strict Schema Adherence**: Do not emit custom events. If you need to log internal state, use `diagnostic` or `op_end` details.
2.  **Capability Flags**: Backends should accurately report their capabilities in `backend_capabilities`. The harness/aggregator should respect these flags to skip unsupported tests gracefully.
3.  **Deterministic Ops**: Avoid `sleep` in backend implementations unless simulating specific timing faults. Use condition variables or Service/Action events for synchronization.
4.  **Unified Ops Interface**: The `exec_op` dispatch pattern works well. Ensure all ops return a standard Result JSON object to be embedded in `op_end`.

## Production Workflow for Contract Verification

1.  **Baseline Lock**: Keep `scenarios_H.json` (Hello World) as the primary CI gate. Run a "Baseline Lock" (docker H00 for ~20 iterations + strict evidence gate) before implementing any new scenario ops to ensure the harness itself is stable.
2.  **Matrix Verification**: Run 5-20 iterations of the full probe matrix on a single distro (e.g., Jazzy) before merging any backend changes.
2.  **Incremental Implementation**: When implementing new ops (e.g., in `backend_prod`), verify against the probe first (TDD).
3.  **Evidence Trustworthiness**:
    *   **Validity Gates**: Never trust a test result if the evidence file is empty or malformed. Classify as `INFRA_FAIL`.
    *   **Single Source**: The `trace.jsonl` is the source of truth for latencies and sequencing. The `report.json` is the source of truth for pass/fail verdicts.
    *   **Forensics**: If a stable probe starts failing, check for "Schema Drift" (backend emitting wrong events) first, then "Semantic Drift" (real ROS behavior change).

