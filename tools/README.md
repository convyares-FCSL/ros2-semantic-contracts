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
