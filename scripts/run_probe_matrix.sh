#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Probe Matrix Runner v2
#
# Builds once per distro, then runs all backend/probe combinations.
# Aggregates results at the end.
# ==============================================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

DISTROS=(humble jazzy)
BACKENDS=(prod rclpy)
PROBES=(P04 L05 S11)

# Clean previous evidence (optional, maybe safe to keep but let's allow overwrite)
# rm -rf evidence/
mkdir -p evidence

echo "=========================================="
echo "Probe Matrix Runner v2"
echo "=========================================="

# Parse arguments or env var
ITERATIONS=${PROBE_ITERS:-5}
if [[ $# -gt 0 ]]; then
    ITERATIONS=$1
fi

echo "Running ${ITERATIONS} iterations per cell..."

# Main Loop: Distro -> Build -> Iteration -> Backend -> Probe
for distro in "${DISTROS[@]}"; do
    echo "=========================================="
    echo ">> preparing ${distro}..."
    echo "=========================================="
    
    # Export ROS_DISTRO for docker-compose volume substitution
    export ROS_DISTRO="${distro}"
    
    # Clean cache explicitly by removing subdirectories
    echo "   Cleaning target-core inside harness container..."
    docker compose run --rm harness bash -c "pwd && ls -F harness/core/target && rm -rf harness/core/target/release harness/core/target/debug && echo 'Cleaned.' && ls -F harness/core/target"
    
    # Build for this distro (using exported ROS_DISTRO)
    if ! docker compose build harness > build_${distro}.log 2>&1; then
        echo "ERROR: Build failed for ${distro}. See build_${distro}.log"
        cat build_${distro}.log
        exit 1
    fi
    echo "Image built for ${distro}."

    for i in $(seq 1 "${ITERATIONS}"); do
        echo ">> Iteration ${i}/${ITERATIONS} for ${distro}..."
        
        for backend in "${BACKENDS[@]}"; do
            for probe in "${PROBES[@]}"; do
                CELL_ID="${distro}/${backend}/${probe}"
                RUN_ID="run_${i}"
                
                # Directory: evidence/<distro>/<backend>/<probe>/<run_id>/
                OUT_DIR="evidence/${CELL_ID}/${RUN_ID}"
                mkdir -p "${OUT_DIR}"
                
                # Run harness
                echo "   Running: ${CELL_ID} (${RUN_ID})"
                
                # Export variables for docker-compose substitution (command & volumes)
                export ORACLE_BACKEND="${backend}"
                export SCENARIO_BUNDLE="scenarios/probes/scenarios_${probe}.json"
                export TRACE="/${OUT_DIR}/trace.jsonl"
                export REPORT="/${OUT_DIR}/report.json"
                export RCLPY_FIDELITY_MODE="strict"
                
                # Capture stdout/stderr
                LOGfile="${OUT_DIR}/stdout.log"
                
                set +e
                docker compose run --rm \
                    --name "harness_${distro}_${backend}_${probe}_${i}" harness > "${LOGfile}" 2>&1
                EXIT_CODE=$?
                set -e
                
                if [[ ${EXIT_CODE} -ne 0 ]]; then
                    echo "   Warning: Harness exited with ${EXIT_CODE}. Check logs."
                fi
            done
        done
    done
done

# 3. Aggregation Phase
echo ">> Aggregating results..."
if ./scripts/aggregate_results.py; then
    echo "Aggregation successful."
else
    echo "Aggregation script failed."
fi

echo "=========================================="
echo "Matrix run complete."
echo "Summary: evidence/aggregate.md"
echo "JSON:    evidence/aggregate.json"
echo "=========================================="

# Check if any failures occurred based on aggregate.json
if grep -q '"failed": [1-9]' evidence/aggregate.json 2>/dev/null; then
    echo "FAILURES DETECTED."
    exit 1
fi

exit 0
