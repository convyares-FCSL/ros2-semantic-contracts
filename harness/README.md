# Oracle Harness (Phase 8)

This harness validates the repository’s semantic contracts against a live ROS 2 implementation.

## Sequencing (C: A → B)

### A) Mechanistic oracle (rclcpp only)
Goal: validate action semantics with minimal confounders.

- Single process, rclcpp.
- One action server + one action client.
- Deterministic scripted scenarios:
  - send_goal accept/reject (“no ghosts”)
  - status transition envelope
  - monotonic status/feedback sequences (as observable)
  - terminal immutability
  - cancel eligibility + cancel race legality (validation, not selection)
- Outputs:
  - machine-readable trace (JSONL) with timestamps + event types
  - summarized verdicts per scenario (pass/fail + reason)

### B) System oracle (Nav2)
Goal: confirm semantics are not contradicted when orchestration/policy layers exist.

- Minimal Nav2 bringup in container.
- Exercise action interactions indirectly (Nav2 stacks).
- Compare observed traces against the same semantic invariants where applicable.
- Record “expected divergence” items explicitly (policy/executor choices that are out-of-scope for core).

This harness vendors third-party headers (e.g. nlohmann/json) to ensure reproducible, offline, audit-safe builds. No network access is required to build or run the oracle.

## Directory layout
- `docker/`  Dockerfiles and compose
- `src/`     rclcpp oracle sources (A), later Nav2 runners (B)
- `configs/` launch/config artifacts, scenario definitions
