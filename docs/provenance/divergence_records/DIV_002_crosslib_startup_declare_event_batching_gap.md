# DIV_002 — Cross-Library Startup Declare Event Batching Gap

- Date: 2026-02-10
- Type: `GAP`
- Spec/Scenario: `SPEC_P05`, `P32_startup_declare_event_batching`
- Related upstream:
  - `ros2/rclcpp#1970` (open, created 2022-07-12): https://github.com/ros2/rclcpp/issues/1970

## 1) Core Gap

Issue-equivalent path tested:
- Node startup performs multiple explicit `declare_parameter` calls.

Expected behavior:
- Startup declaration burst is emitted as one batched `ParameterEvent`.

Observed behavior:
- One event per declared key.
- For 3 declarations: `event_count=3`, `max_keys_in_single_event=1`, `batched_single_event_observed=false`.

## 2) Upstream Evidence and History

`rclcpp#1970` tracks batching startup parameter events as an open feature request.

Useful context from upstream thread:
- Current declaration path publishes immediately per call.
- A transaction-like startup batching API has been discussed but is not merged.
- Gap is architectural and not limited to one client library; same pattern is expected in `rclpy` path as well.

Status as of 2026-02-10:
- Upstream issue remains open.

## 3) Matrix Evidence (Pinned Docker)

| Distro | Backend | Result | Trace signature |
|---|---|---|---|
| Jazzy | `prod` (`rclcpp`) | FAIL | `event_count=3`, `max_keys_in_single_event=1` |
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

- Startup can flood `/parameter_events` for parameter-heavy nodes.
- Observers/tooling see incremental partial startup state rather than one consolidated snapshot.
- Increases bring-up overhead and downstream event processing load.

## 5) Disposition

- Repository disposition: issue-equivalent cross-library gap reproduced and tracked.
- Upstream disposition: open feature gap.
