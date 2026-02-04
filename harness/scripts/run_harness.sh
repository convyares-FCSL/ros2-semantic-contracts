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

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Detect if user provided explicit paths
USER_PROVIDED_TRACE=0
USER_PROVIDED_REPORT=0
if [[ $# -ge 2 ]]; then
    USER_PROVIDED_TRACE=1
fi
if [[ $# -ge 3 ]]; then
    USER_PROVIDED_REPORT=1
fi

# Default base paths (will be per-bundle in default mode)
TRACE_BASE="${2:-/tmp/oracle_trace.jsonl}"
REPORT_BASE="${3:-/tmp/oracle_report.json}"

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
  *)
    echo "ERROR: Unknown ORACLE_BACKEND='${ORACLE_BACKEND}'"
    echo "Supported: stub, ros, prod"
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
    echo "MODE:      Per-bundle evidence"
else
    echo "SCENARIOS: ${TARGETS[0]}"
    echo "TRACE:     ${TRACE_BASE}"
    echo "REPORT:    ${REPORT_BASE}"
fi
echo "BACKEND:   ${ORACLE_BACKEND}"
echo

# ---- Build ----

echo "== Build backend =="
if [[ "${ORACLE_BACKEND}" == "prod" ]]; then
    # Build using colcon from root
    # Ensure setup.bash is sourced by the caller or docker env
    ( cd "${ROOT}" && colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release )
else
    ( cd "${BACKEND_DIR}" && cargo build --release )
fi
echo

echo "== Build core =="
( cd "${CORE_DIR}" && cargo build --release )
echo

# ---- Verify binaries ----

[[ -x "${BACKEND_BIN}" ]] || { echo "ERROR: backend binary missing: ${BACKEND_BIN}"; exit 2; }
[[ -x "${CORE_BIN}" ]] || { echo "ERROR: core binary missing: ${CORE_BIN}"; exit 2; }

# ---- Run ----

FAILED=0
EXPECTED_EVIDENCE=()

for BUNDLE in "${TARGETS[@]}"; do
    NAME=$(basename "${BUNDLE}")
    
    # Determine per-bundle paths
    if [[ $# -eq 0 ]]; then
        # Default mode: per-bundle evidence
        BUNDLE_STEM="${NAME%.json}"  # e.g., scenarios_H
        TRACE_DIR=$(dirname "${TRACE_BASE}")
        REPORT_DIR=$(dirname "${REPORT_BASE}")
        
        # Ensure directories exist
        mkdir -p "${TRACE_DIR}" "${REPORT_DIR}"
        
        TRACE="${TRACE_DIR}/trace_${BUNDLE_STEM}.jsonl"
        REPORT="${REPORT_DIR}/report_${BUNDLE_STEM}.json"
        
        EXPECTED_EVIDENCE+=("${TRACE}")
        EXPECTED_EVIDENCE+=("${REPORT}")
        
        echo "== Run core for ${NAME} =="
        echo "   TRACE:  ${TRACE}"
        echo "   REPORT: ${REPORT}"
    else
        # User-provided mode: use explicit paths
        TRACE="${TRACE_BASE}"
        REPORT="${REPORT_BASE}"
        
        # Ensure directories exist
        mkdir -p "$(dirname "${TRACE}")" "$(dirname "${REPORT}")"
        
        echo "== Run core for ${NAME} =="
    fi
    
    if [[ "${ORACLE_BACKEND}" == "prod" ]]; then
        # prod backend requires an external peer — launch_peer.sh owns the
        # peer lifecycle (start → sentinel gate → oracle_core → teardown).
        SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        "${SCRIPTS_DIR}/launch_peer.sh" "${BUNDLE}" "${TRACE}" "${REPORT}" "${BACKEND_BIN}" "${CORE_BIN}" || FAILED=1
    else
        "${CORE_BIN}" "${BUNDLE}" "${TRACE}" "${REPORT}" --backend "${BACKEND_BIN}" || FAILED=1
    fi
    echo
done

# ---- Self-check for default mode ----

if [[ $# -eq 0 ]]; then
    echo "== Self-check: verifying per-bundle evidence =="
    SELF_CHECK_FAILED=0
    
    # Check that trace files exist (should always be created)
    for ((i=0; i<${#EXPECTED_EVIDENCE[@]}; i+=2)); do
        TRACE_FILE="${EXPECTED_EVIDENCE[i]}"
        if [[ ! -s "${TRACE_FILE}" ]]; then
            echo "ERROR: Expected trace file missing or empty: ${TRACE_FILE}"
            SELF_CHECK_FAILED=1
        else
            echo "✓ ${TRACE_FILE}"
        fi
    done
    
    # Check report files (may be missing if bundle failed)
    for ((i=1; i<${#EXPECTED_EVIDENCE[@]}; i+=2)); do
        REPORT_FILE="${EXPECTED_EVIDENCE[i]}"
        if [[ -s "${REPORT_FILE}" ]]; then
            echo "✓ ${REPORT_FILE}"
        else
            echo "⚠ ${REPORT_FILE} (missing or empty - bundle may have failed)"
        fi
    done
    
    if [[ ${SELF_CHECK_FAILED} -ne 0 ]]; then
        echo "FAILURE: Self-check failed (missing trace files)."
        exit 1
    fi
    echo
    
    echo "Evidence files:"
    for EVIDENCE_FILE in "${EXPECTED_EVIDENCE[@]}"; do
        if [[ -f "${EVIDENCE_FILE}" ]]; then
            echo "  ${EVIDENCE_FILE}"
        fi
    done
else
    echo "Trace:  ${TRACE_BASE}"
    echo "Report: ${REPORT_BASE}"
fi

if [[ ${FAILED} -ne 0 ]]; then
    echo "FAILURE: One or more bundles failed."
    exit 1
fi

exit 0

