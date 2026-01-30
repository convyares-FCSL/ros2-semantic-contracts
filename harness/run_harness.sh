#!/usr/bin/env bash
set -euo pipefail

SCENARIOS="${1:-configs/scenarios.example.json}"
TRACE="${2:-/tmp/ros2_semantic_trace.jsonl}"

ORACLE_BACKEND="${ORACLE_BACKEND:-stub}"

cmake -S . -B build >/dev/null
cmake --build build >/dev/null

if [[ "$ORACLE_BACKEND" == "ros" ]]; then
  ./build/oracle_a_ros "$SCENARIOS" "$TRACE"
else
  ./build/oracle_a "$SCENARIOS" "$TRACE"
fi

./build/audit_trace "$TRACE"
echo "Trace: $TRACE"
