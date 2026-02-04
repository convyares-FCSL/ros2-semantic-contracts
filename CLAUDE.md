# Claude Code Rules — ros2-semantic-contracts

## Non-negotiables
- No simulated/faked backend behavior to satisfy scenarios.
- Trace schema is fixed: only approved event types (see `harness/schemas/trace_event.schema.json`). Fields `ok`, `pass`, `fail`, `verdict`, `assertion` are forbidden. Outcomes must be encoded in `op_end.detail`.
- Exit codes:
  - oracle_core: 0 = all pass, 1 = scenario(s) FAIL, 2 = bundle error, 4 = system error
  - backends: only 0/2/4 (never 1; pass/fail is determined by oracle, not backend)
- Public headers must not leak internal/3rdparty deps (header_sanity must pass).
- Preserve evidence: never overwrite multi-bundle evidence without per-bundle separation.
- H00 (baseline smoke scenario) must remain stable; regressions block all other work.
- Results must be deterministic: 5/5 identical outcomes in Docker validation.
- Axx scenarios must declare `boundary` metadata (sut/peer). See `docs/provenance/oracle/boundaries.md`.

## Scope
- backend_prod (rclcpp) is production-grade C++20.
- Probe backends (e.g., rclpy) are for divergence discovery only and must be "strict fidelity":
  - Must call real ROS 2 client library APIs (rclpy, rclrs, etc.)
  - No mocking, no hard-coded traces, no simulated accept/reject behavior
- New ops go in `harness/backends/backend_prod/src/ops/`; follow the dispatch pattern in `backend.cpp`.

## Working style
- Make minimal diffs, no unrelated refactors.
- Prefer extending existing ops patterns.
- If you don’t know a constraint, stop and ask rather than guessing.

## Read first
- docs/CONTEXT.md
- README.md
- docs/spec/*
- docs/provenance/scenario/*
- harness/schemas/*.json (scenario and trace validation)
