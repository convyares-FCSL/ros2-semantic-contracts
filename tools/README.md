# Tools

Developer utilities for the ros2-semantic-contracts project.

## Directory Structure

```
tools/
├── analysis/           # Result aggregation and analysis
│   └── aggregate_results.py
├── ci/                 # CI/CD utilities
│   └── provenance_gate.py
└── testing/            # Test runners
    └── run_reliability.sh
```

## Common Tasks

### Run reliability tests

Run multiple iterations of the H-series scenarios to verify determinism:

```bash
./tools/testing/run_reliability.sh       # default 5 iterations
./tools/testing/run_reliability.sh 20    # 20 iterations
N=10 ./tools/testing/run_reliability.sh  # via environment
ROS_DISTRO=humble ./tools/testing/run_reliability.sh 3
# Explicit pinned base image override:
ROS_DISTRO=humble ROS_BASE_IMAGE=ros:humble-ros-base@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8 ./tools/testing/run_reliability.sh 3
# Exploratory-only (unpinned) override:
ALLOW_UNPINNED_BASE_IMAGE=1 ROS_DISTRO=humble ROS_BASE_IMAGE=ros:humble-ros-base ./tools/testing/run_reliability.sh 3
```

Results are written to `evidence/reliability/` with per-run traces and an aggregate summary.

### Aggregate results

Aggregate evidence from reliability runs into a summary report:

```bash
python3 tools/analysis/aggregate_results.py
```

Output: `evidence/aggregate.md`

### Check provenance gate

Verify scenario provenance for CI:

```bash
python3 tools/ci/provenance_gate.py
```

## Note

Internal harness scripts (used by the oracle during execution) are located in `harness/scripts/`.
