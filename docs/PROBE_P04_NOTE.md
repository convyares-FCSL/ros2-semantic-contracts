# P04 Probe Forensics: From Failure to Pass

## Summary
The **P04 (No Partial Application)** probe tests that `set_parameters_atomically` (or batch set) correctly rejects a batch update if *any* parameter in the batch is invalid (e.g., undeclared), ensuring atomic "all-or-nothing" semantics.

**Current Status**: ✅ PASS on both `backend_prod` (C++) and `backend_rclpy` (Python).

## Historical Failures: Root Cause Analysis

### 1. Backend Prod (C++) Failure
*   **Cause**: **Missing Implementation**.
*   **Details**: The initial `backend_prod` was a skeleton that did not verify parameter operations. It lacked the `ops::exec_declare_param` and `ops::exec_set_params_batch` dispatchers.
*   **Resolution**: Implemented these operations using `rclcpp::Node::declare_parameter` and `rclcpp::Node::set_parameters_atomically` (verified in `src/ops/params.cpp`).
*   **Verification**: The pass confirms that `rclcpp` correctly implements atomic batch updates by default.

### 2. Backend Rclpy (Python) Failure
*   **Cause**: **Schema Mismatch (Trace Validity)**.
*   **Details**: The `rclpy` backend initially emitted custom trace events like `param_set_batch_response`. The Oracle Schema strictly requires all operations to log results within `op_end` details. When the harness was strictified, P04 expectations were updated to look for `op_end`, but the backend was still emitting custom events (or the harness was rejecting them as "Forbidden").
*   **Resolution**: Refactored `backend_rclpy/ops/params.py` to remove custom emitters. Confirmed use of `self.node.set_parameters_atomically(param_list)`.

*   **Verification**: The pass confirms that `rclpy` atomic parameter setting works as expected and is now consistently observable via the standard schema.

## Sanity Check
To ensure we aren't "fake passing" (e.g., asserting trivial success), the runner now sanity-checks the report to ensure `all_successful: false` is explicitly part of the observed result for the mixed-validity batch.
