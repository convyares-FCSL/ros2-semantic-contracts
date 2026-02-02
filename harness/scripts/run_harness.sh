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
#   If no scenario_bundle is provided, runs sequence: H, S, P, L, A.
#
# Backend selection:
#   ./scripts/run_harness.sh                      # defaults to ORACLE_BACKEND=ros
#   ORACLE_BACKEND=stub ./scripts/run_harness.sh
#   ORACLE_BACKEND=ros  ./scripts/run_harness.sh
# -----------------------------------------------------------------------------

# Resolve repo root: two levels up from harness/scripts
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

TRACE="${2:-/tmp/oracle_trace.jsonl}"
REPORT="${3:-/tmp/oracle_report.json}"

# Determine target bundles
TARGETS=()
if [[ $# -eq 0 ]]; then
    # Default order: H, S, P, L, A
    TARGETS+=("${ROOT}/harness/scenarios/scenarios_H.json")
    TARGETS+=("${ROOT}/harness/scenarios/scenarios_S.json")
    TARGETS+=("${ROOT}/harness/scenarios/scenarios_P.json")
    TARGETS+=("${ROOT}/harness/scenarios/scenarios_L.json")
    TARGETS+=("${ROOT}/harness/scenarios/scenarios_A.json")
else
    TARGETS+=("${1}")
fi

# Default is now ros
ORACLE_BACKEND="${ORACLE_BACKEND:-ros}"

CORE_DIR="${ROOT}/harness/core"
CORE_BIN="${CORE_DIR}/target/release/oracle_core"

# Backend registry (extend here; avoid redesigning the script later)
BACKEND_DIR=""
BACKEND_BIN=""

case "${ORACLE_BACKEND}" in
  stub)
    BACKEND_DIR="${ROOT}/harness/backends/backend_stub"
    BACKEND_BIN="${BACKEND_DIR}/target/release/backend_stub"
    ;;
  ros)
    BACKEND_DIR="${ROOT}/harness/backends/backend_ros"
    BACKEND_BIN="${BACKEND_DIR}/target/release/backend_ros"
    ;;
  prod)
    BACKEND_DIR="${ROOT}/harness/backends/backend_prod"
    # Assumes colcon build from root output
    BACKEND_BIN="${ROOT}/install/backend_prod/lib/backend_prod/backend_prod"
    ;;
  rclpy)
    BACKEND_DIR="${ROOT}/harness/backends/backend_rclpy"
    BACKEND_BIN="python3 ${BACKEND_DIR}/backend_rclpy.py"
    ;;
  *)
    echo "ERROR: Unknown ORACLE_BACKEND='${ORACLE_BACKEND}'"
    echo "Supported: stub, ros, prod, rclpy"
    exit 2
    ;;
esac

# ---- Preflight ----

for BUNDLE in "${TARGETS[@]}"; do
    if [[ ! -f "${BUNDLE}" ]]; then
      echo "ERROR: Scenario bundle not found: ${BUNDLE}"
      exit 2
    fi
done

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
if [[ $# -eq 0 ]]; then
    echo "SCENARIOS: [Default Sequence: H, S, P, L, A]"
else
    echo "SCENARIOS: ${TARGETS[0]}"
fi
echo "TRACE:     ${TRACE}"
echo "REPORT:    ${REPORT}"
echo "BACKEND:   ${ORACLE_BACKEND}"
echo

# ---- Build ----

echo "== Build backend =="
if [[ "${ORACLE_BACKEND}" == "prod" ]]; then
    # Build using colcon from root
    # Ensure setup.bash is sourced by the caller or docker env
    ( cd "${ROOT}" && colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release )
elif [[ "${ORACLE_BACKEND}" == "rclpy" ]]; then
    echo "Skipping build (Python backend)"
else
    ( cd "${BACKEND_DIR}" && cargo build --release )
fi
echo

echo "== Build core =="
( cd "${CORE_DIR}" && cargo build --release )
echo

# ---- Verify binaries ----

if [[ "${ORACLE_BACKEND}" == "rclpy" ]]; then
    # For Python backend, just check file exists
    [[ -f "${ROOT}/harness/backends/backend_rclpy/backend_rclpy.py" ]] || { echo "ERROR: backend script missing"; exit 2; }
else
    [[ -x "${BACKEND_BIN}" ]] || { echo "ERROR: backend binary missing: ${BACKEND_BIN}"; exit 2; }
fi
[[ -x "${CORE_BIN}" ]] || { echo "ERROR: core binary missing: ${CORE_BIN}"; exit 2; }

# ---- Run ----

FAILED=0

for BUNDLE in "${TARGETS[@]}"; do
    NAME=$(basename "${BUNDLE}")
    echo "== Run core for ${NAME} =="
    "${CORE_BIN}" "${BUNDLE}" "${TRACE}" "${REPORT}" --backend "${BACKEND_BIN}" || FAILED=1
    echo
done

echo "Trace:  ${TRACE}"
echo "Report: ${REPORT}"

if [[ ${FAILED} -ne 0 ]]; then
    echo "FAILURE: One or more bundles failed."
    exit 1
fi

exit 0
