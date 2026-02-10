# DIV_003 — `rclpy` Jazzy Regression on CLI Override + Typed No-Default Declare

- Date: 2026-02-10
- Type: `REGRESSION`
- Spec/Scenario: `SPEC_PC11`, `P31_no_default_with_startup_override`
- Related upstream:
  - `ros2/rclpy#1460` (open, created 2025-05-15): https://github.com/ros2/rclpy/issues/1460
  - Discussed introducing change: PR `ros2/rclpy#1216` (context in issue thread)
  - `rclcpp` static-typed note reference: https://docs.ros.org/en/rolling/p/rclcpp/doc/notes_on_statically_typed_parameters.html

## 1) Core Regression

Exact path tested:
- CLI override: `--ros-args -p key:=value`
- Then typed declaration without default (`descriptor.type` set, no default value passed)

Expected behavior:
- Declaration succeeds and binds provided CLI override value.

Observed behavior:
- Jazzy `rclpy` fails with:
  - `Cannot declare a statically typed parameter with default value of type PARAMETER_NOT_SET`

## 2) Upstream Evidence and History

`rclpy#1460` documents this mismatch vs `rclcpp` and reports breakage during Humble -> Jazzy upgrade.

Useful context from upstream discussion:
- Report links behavior to stricter type handling changes associated with `rclpy#1216`.
- The failure path is consistent with evaluating "no default" before consuming startup override.
- `rclcpp` remains compatible with typed no-default declaration when override is provided.

Status as of 2026-02-10:
- Upstream issue remains open.

## 3) Matrix Evidence (Pinned Docker)

| Distro | Backend | Result | Trace signature |
|---|---|---|---|
| Jazzy | `prod` (`rclcpp`) | PASS | declaration succeeds with CLI override |
| Jazzy | `py` (`rclpy`) | FAIL | throws `PARAMETER_NOT_SET` declaration error |
| Humble | `prod` (`rclcpp`) | PASS | succeeds |
| Humble | `py` (`rclpy`) | PASS | succeeds |

Evidence files:
- `evidence/reliability/jazzy/prod/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/report.json`
- `evidence/reliability/humble/prod/run_01/report.json`
- `evidence/reliability/humble/py/run_01/report.json`
- `evidence/reliability/jazzy/py/run_01/trace.jsonl`
- `evidence/reliability/humble/py/run_01/trace.jsonl`

## 4) Impact

- Python nodes relying on required startup parameters may fail only on Jazzy `rclpy`.
- Cross-language parity breaks for the same node configuration contract.
- Migration risk: Humble-validated Python nodes can fail at Jazzy startup.

## 5) Disposition

- Repository disposition: exact issue path reproduced, regression confirmed in matrix.
- Upstream disposition: unresolved (`ros2/rclpy#1460` open).
- Operational note: the practical workaround is supplying placeholder defaults, which weakens the "required-at-startup" parameter pattern.
