#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Oracle Harness runner (v0.1)
#
# - Builds backend + core with visible logs
# - Runs core (core invokes backend)
# - Prints trace/report locations
#
# Usage:
#   ./scripts/run_harness.sh [scenario_bundle.json] [trace.jsonl] [report.json]
#
# Backend selection:
#   ORACLE_BACKEND=stub ./scripts/run_harness.sh
#   ORACLE_BACKEND=ros  ./scripts/run_harness.sh   # (future; adapter not yet present)
# -----------------------------------------------------------------------------

# Resolve repo root: two levels up from harness/scripts
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

SCENARIOS="${1:-${ROOT}/harness/scenarios/scenarios_H.json}"
TRACE="${2:-/tmp/oracle_trace.jsonl}"
REPORT="${3:-/tmp/oracle_report.json}"

ORACLE_BACKEND="${ORACLE_BACKEND:-stub}"

CORE_DIR="${ROOT}/harness/core"
CORE_BIN="${CORE_DIR}/target/release/oracle_core"

STUB_DIR="${ROOT}/harness/backends/backend_stub"

# Backend registry (extend here; avoid redesigning the script later)
BACKEND_DIR=""
BACKEND_BIN=""

case "${ORACLE_BACKEND}" in
  stub)
    BACKEND_DIR="${STUB_DIR}"
    BACKEND_BIN="${BACKEND_DIR}/target/release/backend_stub"
    ;;
  # ros)
  #   BACKEND_DIR="${ROOT}/harness/backends/backend_ros_rclcpp"
  #   BACKEND_BIN="${BACKEND_DIR}/target/release/backend_ros_rclcpp"
  #   ;;
  *)
    echo "ERROR: Unknown ORACLE_BACKEND='${ORACLE_BACKEND}'"
    echo "Supported: stub"
    exit 2
    ;;
esac

# ---- Preflight ----

if [[ ! -f "${SCENARIOS}" ]]; then
  echo "ERROR: Scenario bundle not found: ${SCENARIOS}"
  exit 2
fi

if [[ ! -d "${CORE_DIR}" ]]; then
  echo "ERROR: Core directory not found: ${CORE_DIR}"
  exit 2
fi

if [[ ! -d "${BACKEND_DIR}" ]]; then
  echo "ERROR: Backend directory not found: ${BACKEND_DIR}"
  exit 2
fi

echo "== Oracle Harness =="
echo "ROOT:      ${ROOT}"
echo "SCENARIOS: ${SCENARIOS}"
echo "TRACE:     ${TRACE}"
echo "REPORT:    ${REPORT}"
echo "BACKEND:   ${ORACLE_BACKEND}"
echo

# ---- Build ----

echo "== Build backend =="
( cd "${BACKEND_DIR}" && cargo build --release )
echo

echo "== Build core =="
( cd "${CORE_DIR}" && cargo build --release )
echo

# ---- Verify binaries ----

[[ -x "${BACKEND_BIN}" ]] || { echo "ERROR: backend binary missing: ${BACKEND_BIN}"; exit 2; }
[[ -x "${CORE_BIN}" ]] || { echo "ERROR: core binary missing: ${CORE_BIN}"; exit 2; }

# ---- Run ----

echo "== Run core (invokes backend) =="
"${CORE_BIN}" "${SCENARIOS}" "${TRACE}" "${REPORT}" --backend "${BACKEND_BIN}"
RC=$?

echo
echo "Trace:  ${TRACE}"
echo "Report: ${REPORT}"
exit "${RC}"
