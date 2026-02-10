# DIV_001 — Cross-Library `get_parameter_types` Contract Violation

- Date: 2026-02-10
- Type: `DEFECT`
- Spec/Scenario: `SPEC_PC12`, `P24_get_parameter_types_response_length`
- Related upstream:
  - `ros2/rclcpp#2512` (open, created 2024-04-23): https://github.com/ros2/rclcpp/issues/2512
  - Related tooling impact: `ros2/ros2cli#924` https://github.com/ros2/ros2cli/issues/924

## 1) Core Violation

Expected contract behavior:
- `GetParameterTypes` response preserves 1:1 request mapping.
- Undeclared names are returned as `PARAMETER_NOT_SET` (`0`) entries.

Observed behavior on both measured libraries:
- If any requested name is undeclared, the call fails as a whole.
- Mixed request (`declared + undeclared`) returns `successful=false`, `count=0`, `types=[]`.

Example trace signature:
- `request_count=3`
- `count=0`
- `reason=NotDeclared`

## 2) Upstream Evidence and History

Issue thread (`rclcpp#2512`) explicitly documents mismatch between `rcl_interfaces` service contract text and current `rclcpp`/`rclpy` behavior.

Useful context from upstream discussion:
- Current implementation treats undeclared parameter in list as request-level error.
- Aligning behavior with strict 1:1 response mapping is a behavior change with compatibility impact.
- CLI tooling confusion is tracked in `ros2/ros2cli#924`.

Status as of 2026-02-10:
- Upstream issue remains open.

## 3) Matrix Evidence (Pinned Docker)

| Distro | Backend | Result | Trace signature |
|---|---|---|---|
| Jazzy | `prod` (`rclcpp`) | FAIL | `request_count=3`, `count=0`, `reason=NotDeclared` |
| Jazzy | `py` (`rclpy`) | FAIL | same failure mode |
| Humble | `prod` (`rclcpp`) | FAIL | same failure mode |
| Humble | `py` (`rclpy`) | FAIL | same failure mode |

Evidence files:
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/report.json`
- `evidence/reliability/jazzy/prod/run_01/trace.jsonl`
- `evidence/reliability/jazzy/py/run_01/trace.jsonl`

## 4) Impact

- Batched type introspection is unsafe if callers assume request/response index alignment.
- A single undeclared key can invalidate entire multi-key query.
- Extra retry logic (single-key probing) is required for robust clients.

## 5) Disposition

- Repository disposition: confirmed cross-library contract violation, tracked as divergence.
- Upstream disposition: unresolved (open issue).
