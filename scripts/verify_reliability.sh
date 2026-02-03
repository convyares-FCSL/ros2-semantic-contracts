#!/usr/bin/env bash
set -e

# Default values
COUNT=20
DISTRO="humble"
BACKEND="rclpy"
PROBE="L05"

usage() {
    echo "Usage: $0 [-n count] [distro] [backend] [probe]"
    echo "  -n count : Number of runs (default: 20)"
    echo "  distro   : ROS distro (humble|jazzy, default: humble)"
    echo "  backend  : Oracle backend (prod|rclpy|ros|stub, default: rclpy)"
    echo "  probe    : Probe ID (L05|S11|P04|etc, default: L05)"
    exit 1
}

while getopts "n:h" opt; do
    case $opt in
        n) COUNT="$OPTARG" ;;
        h) usage ;;
        *) usage ;;
    esac
done
shift $((OPTIND-1))

if [ "$#" -ge 1 ]; then DISTRO="$1"; fi
if [ "$#" -ge 2 ]; then BACKEND="$2"; fi
if [ "$#" -ge 3 ]; then PROBE="$3"; fi

# Resolve paths
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUNDLE="${ROOT}/harness/scenarios/probes/scenarios_${PROBE}.json"

if [ ! -f "$BUNDLE" ]; then
    echo "Error: Bundle not found: $BUNDLE"
    exit 1
fi

echo "=========================================="
echo "Reliability Verification"
echo "=========================================="
echo "Runs:    $COUNT"
echo "Cell:    $DISTRO / $BACKEND / $PROBE"
echo "Bundle:  $BUNDLE"
echo "=========================================="

# Build once
echo "[Build] Building docker image for $DISTRO..."
export ROS_DISTRO="$DISTRO"
docker compose build --build-arg ROS_DISTRO="$DISTRO" > /dev/null

# Execution loop
PASS=0
FAIL=0
ERROR=0

echo "[Run] Starting execution loop..."

for i in $(seq 1 $COUNT); do
    printf "Run %02d/%02d: " "$i" "$COUNT"
    
    # Define non-overlapping trace paths for safety (though serial)
    export ORACLE_BACKEND="$BACKEND"
    export SCENARIO_BUNDLE="scenarios/probes/scenarios_${PROBE}.json"
    export TRACE="/evidence/${DISTRO}/${BACKEND}/${PROBE}/run_${i}_trace.jsonl"
    export REPORT="/evidence/${DISTRO}/${BACKEND}/${PROBE}/run_${i}_report.json"
    
    # Run harness (capture output)
    set +e
    OUTPUT=$(docker compose run --rm harness 2>&1)
    EXIT_CODE=$?
    set -e
    
    # Check result
    if [ $EXIT_CODE -ne 0 ]; then
        echo "ERROR (Ext Code $EXIT_CODE)"
        echo "$OUTPUT"
        ERROR=$((ERROR + 1))
    else
        # Inspect output for FAIL/PASS
        # The harness prints "FAIL ..." or "1 TEST(S) FAILED" if failed.
        # But wait, harness exit code IS 1 if failed.
        # harness/scripts/run_harness.sh: `if [[ ${FAILED} -ne 0 ]]; then exit 1; fi`
        
        # So exit code 1 means FAIL or ERROR.
        # If exit code 0, it means PASS.
        
        # However, run_probe_matrix.sh treats non-zero as "Failed".
        # But here exit code captures usage errors too.
        # Assuming harness works, exit 1 = Verification Failed.
        
        # Wait, if output contains "FAIL", it failed.
        echo "PASS"
        PASS=$((PASS + 1))
    fi
    
    if [ $EXIT_CODE -ne 0 ]; then
        # Check if it was a build failure or test failure?
        if echo "$OUTPUT" | grep -q "TEST(S) FAILED"; then
            # Test failure
            # Wait, console output might get suppressed by OUTPUT capture.
            # I captured it.
            echo "FAIL"
            FAIL=$((FAIL + 1))
            # Decrement ERROR incremented above?
            ERROR=$((ERROR - 1)) 
        elif echo "$OUTPUT" | grep -q "FAILURE: One or more bundles failed"; then
             echo "FAIL"
             FAIL=$((FAIL + 1))
             ERROR=$((ERROR - 1))
        fi
    fi
done

echo "=========================================="
echo "Results:"
echo "  PASS:  $PASS"
echo "  FAIL:  $FAIL"
echo "  ERROR: $ERROR"
echo "=========================================="

if [ "$FAIL" -gt 0 ] || [ "$ERROR" -gt 0 ]; then
    exit 1
fi
exit 0
