#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Probe Matrix Runner
#
# Runs probe scenarios across all combinations of:
#   - Distros: humble, jazzy
#   - Backends: prod (rclcpp), rclpy
#   - Probes: P04, L05, S11
#
# For each cell (12 total):
#   1. Build docker image for distro
#   2. Run harness with appropriate env vars
#   3. Verify evidence artifacts exist
#   4. Extract headline metric (PASS/FAIL/SKIP)
#
# Usage:
#   ./scripts/run_probe_matrix.sh
#
# Evidence artifacts written to: evidence/<distro>/<backend>/<probe>/
# ==============================================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

DISTROS=(humble jazzy)
BACKENDS=(prod rclpy)
PROBES=(P04 L05 S11)

FAILED=0
TOTAL=0
PASSED=0

echo "=========================================="
echo "Probe Matrix Runner"
echo "=========================================="
echo "Distros:  ${DISTROS[*]}"
echo "Backends: ${BACKENDS[*]}"
echo "Probes:   ${PROBES[*]}"
echo "Total cells: $((${#DISTROS[@]} * ${#BACKENDS[@]} * ${#PROBES[@]}))"
echo "=========================================="
echo

for distro in "${DISTROS[@]}"; do
  for backend in "${BACKENDS[@]}"; do
    for probe in "${PROBES[@]}"; do
      TOTAL=$((TOTAL + 1))
      CELL_ID="${distro}/${backend}/${probe}"
      
      echo "[$TOTAL] Running: ${CELL_ID}"
      echo "  Distro: ${distro}, Backend: ${backend}, Probe: ${probe}"
      
      # Setup environment variables
      export ROS_DISTRO="${distro}"
      export ORACLE_BACKEND="${backend}"
      export SCENARIO_BUNDLE="scenarios/probes/scenarios_${probe}.json"
      export TRACE="/evidence/${distro}/${backend}/${probe}/trace.jsonl"
      export REPORT="/evidence/${distro}/${backend}/${probe}/report.json"
      
      # Create evidence directory
      mkdir -p "evidence/${distro}/${backend}/${probe}"
      
      # Build docker image
      echo "  Building docker image for ${distro}..."
      if ! docker compose build --build-arg ROS_DISTRO="${distro}" 2>&1 | grep -v "^#" | tail -n 5; then
        echo "  ERROR: Docker build failed"
        FAILED=$((FAILED + 1))
        continue
      fi
      
      # Run harness
      echo "  Running harness..."
      if ! docker compose run --rm harness 2>&1 | tail -n 20; then
        echo "  ERROR: Harness execution failed"
        FAILED=$((FAILED + 1))
        continue
      fi
      
      # Verify artifacts exist
      if [[ ! -s "evidence/${distro}/${backend}/${probe}/trace.jsonl" ]]; then
        echo "  ERROR: trace.jsonl missing or empty"
        FAILED=$((FAILED + 1))
        continue
      fi
      
      if [[ ! -s "evidence/${distro}/${backend}/${probe}/report.json" ]]; then
        echo "  ERROR: report.json missing or empty"
        FAILED=$((FAILED + 1))
        continue
      fi
      
      # Extract headline metric (PASS/FAIL/SKIP)
      VERDICT=$(jq -r '.scenarios[].verdict // "UNKNOWN"' "evidence/${distro}/${backend}/${probe}/report.json" 2>/dev/null | head -n1 || echo "UNKNOWN")
      
      # Extract key metric from trace
      case "${probe}" in
        L05)
          # Look for transition response with successful=false
          METRIC=$(grep -o '"successful":false' "evidence/${distro}/${backend}/${probe}/trace.jsonl" | wc -l || echo "0")
          echo "  Metric: Rejections=${METRIC}"
          ;;
        S11)
          # Look for get_state latency
          METRIC=$(grep 'get_state_response' "evidence/${distro}/${backend}/${probe}/trace.jsonl" | grep -o '"latency_ms":[0-9]*' | head -n1 || echo "latency_ms:TIMEOUT")
          echo "  Metric: ${METRIC}"
          ;;
        P04)
          # Look for all_successful=false
          METRIC=$(grep 'param_set_batch_response' "evidence/${distro}/${backend}/${probe}/trace.jsonl" | grep -o '"all_successful":[a-z]*' | head -n1 || echo "all_successful:unknown")
          echo "  Metric: ${METRIC}"
          ;;
      esac
      
      echo "  Result: ${VERDICT}"
      
      if [[ "${VERDICT}" == "PASS" ]]; then
        PASSED=$((PASSED + 1))
      fi
      
      echo
    done
  done
done

echo "=========================================="
echo "Matrix Runner Summary"
echo "=========================================="
echo "Total cells:  ${TOTAL}"
echo "Passed:       ${PASSED}"
echo "Failed/Skip:  $((TOTAL - PASSED))"
echo "Build/exec failures: ${FAILED}"
echo "=========================================="

if [[ ${FAILED} -gt 0 ]]; then
  echo "FAILURE: ${FAILED} cells failed to execute"
  exit 1
fi

echo "Matrix run complete. Check evidence/ for detailed results."
exit 0
