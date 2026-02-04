#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Reliability Runner for H-series (H00 + H01) scenarios
#
# Builds docker image once, then runs N iterations with per-run evidence.
# Aggregates results at the end.
#
# Usage:
#   ./scripts/run_reliability.sh           # default N=5
#   ./scripts/run_reliability.sh 20        # N=20
#   N=10 ./scripts/run_reliability.sh      # via environment
# ==============================================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

# Configuration
ITERATIONS="${N:-${1:-5}}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ORACLE_BACKEND="${ORACLE_BACKEND:-prod}"
SCENARIO_BUNDLE="scenarios/scenarios_H.json"

echo "=========================================="
echo "Reliability Runner"
echo "=========================================="
echo "Iterations: ${ITERATIONS}"
echo "Distro:     ${ROS_DISTRO}"
echo "Backend:    ${ORACLE_BACKEND}"
echo "Bundle:     ${SCENARIO_BUNDLE}"
echo "=========================================="

# Clean previous reliability evidence (but preserve other evidence)
EVIDENCE_BASE="evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}"
rm -rf "${EVIDENCE_BASE}"
mkdir -p "${EVIDENCE_BASE}"

# Build docker image once
echo "[Build] Building docker image for ${ROS_DISTRO}..."
export ROS_DISTRO
docker compose build > /dev/null 2>&1 || {
    echo "ERROR: Docker build failed"
    exit 4
}
echo "[Build] Done."

# Execution loop
PASS_COUNT=0
FAIL_COUNT=0
INFRA_FAIL_COUNT=0

echo "[Run] Starting ${ITERATIONS} iterations..."

for i in $(seq 1 "${ITERATIONS}"); do
    RUN_ID="run_$(printf '%02d' "${i}")"
    RUN_DIR="${EVIDENCE_BASE}/${RUN_ID}"
    mkdir -p "${RUN_DIR}"

    printf "  %s: " "${RUN_ID}"

    # Set environment variables for docker-compose
    export ORACLE_BACKEND
    export SCENARIO_BUNDLE
    export TRACE="/evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}/${RUN_ID}/trace.jsonl"
    export REPORT="/evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}/${RUN_ID}/report.json"

    # Run harness and capture output
    LOG_FILE="${RUN_DIR}/stdout.log"
    set +e
    docker compose run --rm harness > "${LOG_FILE}" 2>&1
    EXIT_CODE=$?
    set -e

    # Classify result
    TRACE_FILE="${RUN_DIR}/trace.jsonl"
    REPORT_FILE="${RUN_DIR}/report.json"

    # Check evidence validity gates
    if [[ ! -f "${TRACE_FILE}" ]] || [[ ! -s "${TRACE_FILE}" ]]; then
        echo "INFRA_FAIL (missing/empty trace)"
        INFRA_FAIL_COUNT=$((INFRA_FAIL_COUNT + 1))
        echo "infra_fail: missing/empty trace" > "${RUN_DIR}/result.txt"
        continue
    fi

    if [[ ! -f "${REPORT_FILE}" ]]; then
        echo "INFRA_FAIL (missing report)"
        INFRA_FAIL_COUNT=$((INFRA_FAIL_COUNT + 1))
        echo "infra_fail: missing report" > "${RUN_DIR}/result.txt"
        continue
    fi

    # Validate report JSON
    if ! python3 -c "import json; json.load(open('${REPORT_FILE}'))" 2>/dev/null; then
        echo "INFRA_FAIL (invalid report JSON)"
        INFRA_FAIL_COUNT=$((INFRA_FAIL_COUNT + 1))
        echo "infra_fail: invalid report JSON" > "${RUN_DIR}/result.txt"
        continue
    fi

    # Check outcome from report
    PASS=$(python3 -c "import json; print(json.load(open('${REPORT_FILE}')).get('pass', False))")
    if [[ "${PASS}" == "True" ]]; then
        echo "PASS"
        PASS_COUNT=$((PASS_COUNT + 1))
        echo "pass" > "${RUN_DIR}/result.txt"
    else
        echo "FAIL (semantic)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        echo "semantic_fail" > "${RUN_DIR}/result.txt"
    fi
done

echo "=========================================="
echo "Results Summary:"
echo "  PASS:       ${PASS_COUNT}"
echo "  FAIL:       ${FAIL_COUNT}"
echo "  INFRA_FAIL: ${INFRA_FAIL_COUNT}"
echo "=========================================="

# Run aggregation
echo "[Aggregate] Running aggregation..."
if python3 "${ROOT}/scripts/aggregate_results.py"; then
    echo "[Aggregate] Done. See evidence/aggregate.md"
else
    echo "[Aggregate] Warning: aggregation script had issues"
fi

# Exit with appropriate code
if [[ ${FAIL_COUNT} -gt 0 ]] || [[ ${INFRA_FAIL_COUNT} -gt 0 ]]; then
    exit 1
fi
exit 0
