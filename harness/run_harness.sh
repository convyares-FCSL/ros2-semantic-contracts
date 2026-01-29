#!/usr/bin/env bash
set -euo pipefail

SCENARIOS="${1:-configs/scenarios.example.json}"
TRACE="${2:-/tmp/ros2_semantic_trace.jsonl}"

# Build
cmake -S . -B build >/dev/null
cmake --build build >/dev/null

# Run + audit
./build/oracle_a "$SCENARIOS" "$TRACE"
./build/audit_trace "$TRACE"

echo "Trace: $TRACE"
