#!/usr/bin/env bash
set -euo pipefail

SCENARIOS="${1:-configs/scenarios_a.json}"
TRACE="${2:-/tmp/ros2_semantic_trace.jsonl}"

ORACLE_BACKEND="${ORACLE_BACKEND:-ros}"

cmake -S . -B build >/dev/null
cmake --build build >/dev/null

echo "Oracle backend: ${ORACLE_BACKEND}"
if [[ "${ORACLE_BACKEND}" == "ros" ]]; then
  if [[ ! -x ./build/oracle_a_ros ]]; then
    echo "ERROR: ORACLE_BACKEND=ros but ./build/oracle_a_ros is missing."
    echo "Hint: source your ROS environment and reconfigure:"
    echo "  source /opt/ros/\${ROS_DISTRO}/setup.bash"
    exit 2
  fi

  # Print auditable runtime environment info
  echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"

  ./build/oracle_a_ros "$SCENARIOS" "$TRACE"
else
  if [[ ! -x ./build/oracle_a ]]; then
    echo "ERROR: ./build/oracle_a is missing."
    exit 2
  fi
  ./build/oracle_a "$SCENARIOS" "$TRACE"
fi

./build/audit_trace "$TRACE"
echo "Trace: $TRACE"
