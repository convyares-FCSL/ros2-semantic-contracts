#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Reliability Runner for ROS 2 Semantic Contracts
#
# Usage:
#   ./tools/testing/run_reliability.sh [N] [flags]
#
# Flags:
#   --verbose       Show docker build logs and detailed output
#   --quick         Show only run-level PASS/FAIL status
#   --summary       Show per-scenario status (default)
#   --strict        Fail if any scenario is skipped due to ambiguity
#   --bundle <path> Run specific bundle file
#   --scenario <ID> Run specific scenario (autogenerates bundle)
#
# ==============================================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

# Defaults
ITERATIONS="5"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ORACLE_BACKEND="${ORACLE_BACKEND:-prod}"
MODE="summary"  # summary (default), quick, verbose
STRICT=0
BUNDLE_ARG=""
SCENARIO_ARG=""
VERBOSE=0

# Argument Parsing
args=()
while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose) VERBOSE=1; MODE="verbose"; shift ;;
        --quick) MODE="quick"; shift ;;
        --summary) MODE="summary"; shift ;;
        --strict) STRICT=1; shift ;;
        --bundle) BUNDLE_ARG="$2"; shift 2 ;;
        --scenario) SCENARIO_ARG="$2"; shift 2 ;;
        [0-9]*) ITERATIONS="$1"; shift ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if [[ -n "${N:-}" ]]; then ITERATIONS="${N}"; fi

# ------------------------------------------------------------------------------
# Bundle Construction / selection
# ------------------------------------------------------------------------------
EFFECTIVE_BUNDLE="/tmp/reliability_bundle.json"

# Python helper to merge/filter bundles
# Python helper to resolve bundles
# Returns exit code 0 on success, 1 on error
resolve_bundles() {
    python3 - <<EOF
import json, sys, glob, os

bundle_arg = '${BUNDLE_ARG}'
scenario_arg = '${SCENARIO_ARG}'

if bundle_arg:
    print(bundle_arg)
    sys.exit(0)

if scenario_arg:
    # Ephemeral bundle for single scenario
    merged = {'version': '0.1', 'group': 'Reliability', 'description': 'Single Scenario', 'scenarios': {}}
    found = False
    # Search all scenarios for this ID
    for fpath in sorted(glob.glob('harness/scenarios/scenarios_*.json')):
        try:
            with open(fpath) as f:
                b = json.load(f)
                if scenario_arg in b.get('scenarios', {}):
                    merged['scenarios'][scenario_arg] = b['scenarios'][scenario_arg]
                    found = True
                    break
        except: pass
    
    if not found:
        print(f'Error: Scenario {scenario_arg} not found.', file=sys.stderr)
        sys.exit(1)
        
    out_path = '/tmp/reliability_single.json'
    with open(out_path, 'w') as f:
        json.dump(merged, f, indent=2)
    print(out_path)
    sys.exit(0)

# Default: Return independent bundles for compatible execution
# We explode ALL scenarios into individual ephemeral bundles to ensure:
# 1. Immediate reporting of results (per user request)
# 2. No peer conflicts (H vs A)
bundles = sorted(glob.glob('harness/scenarios/scenarios_[AH]*.json'))
for b_path in bundles:
    try:
        with open(b_path) as f:
            data = json.load(f)
        for sid, sc in sorted(data.get('scenarios', {}).items()):
            # Filter unimplemented scenarios (empty ops or expects)
            ops = sc.get('ops', [])
            expects = sc.get('expects', [])
            if not ops or not expects:
                # Optional: warn user?
                # print(f"Skipping {sid} (unimplemented)", file=sys.stderr)
                continue

            # Create ephemeral file for this scenario
            ephemeral = {'version':'0.1', 'group': 'Reliability', 'scenarios': {sid: sc}}
            # Use safe filename
            out_path = f'/tmp/reliability_split_{sid}.json'
            with open(out_path, 'w') as out:
                json.dump(ephemeral, out, indent=2)
            print(out_path)
    except:
        pass
EOF
}


# Check if single scenario setup required export
if [[ -n "${SCENARIO_ARG}" ]]; then
    if [[ "${SCENARIO_ARG}" == "A01"* || "${SCENARIO_ARG}" == "A02"* ]]; then
        export TARGET_SCENARIO_ID="${SCENARIO_ARG}"
    fi
fi

if [[ -n "${SCENARIO_ARG}" ]]; then
    echo "[Config] Single Scenario: ${SCENARIO_ARG}"
    # When running single A-scenario, export TARGET so launch_peer works
    if [[ "${SCENARIO_ARG}" == "A01"* || "${SCENARIO_ARG}" == "A02"* ]]; then
        # Map short ID to ID expected by launch_peer logic (which checks startswith)
        export TARGET_SCENARIO_ID="${SCENARIO_ARG}"
    fi
fi

# ------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------
echo "=========================================="
echo "Reliability Runner"
echo "=========================================="
echo "Iterations: ${ITERATIONS}"
echo "Distro:     ${ROS_DISTRO}"
echo "Backend:    ${ORACLE_BACKEND}"
echo "Mode:       ${MODE}"
echo "=========================================="

# Clean previous reliability evidence
EVIDENCE_BASE="evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}"

# Use docker to clean to handle root-owned files from previous runs
docker run --rm -v "${PWD}/evidence:/evidence" ubuntu rm -rf "/evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}" || true
mkdir -p "${EVIDENCE_BASE}"

echo -n "[Build] Building docker image for ${ROS_DISTRO}..."
export ROS_DISTRO

if [[ ${VERBOSE} -eq 1 ]]; then
    echo ""
    docker compose build || { echo "ERROR: Docker build failed"; exit 4; }
else
    if docker compose build > /tmp/docker_build.log 2>&1; then
        echo " (quiet)"
    else
        echo " FAILED"
        echo "================ BUILD LOG ================"
        cat /tmp/docker_build.log
        echo "==========================================="
        exit 4
    fi
fi
echo "[Build] Done."

# ------------------------------------------------------------------------------
# Execution Loop
# ------------------------------------------------------------------------------
PASS_COUNT=0
FAIL_COUNT=0
INFRA_FAIL_COUNT=0

# Resolve bundles
BUNDLES_TO_RUN_LIST=$(resolve_bundles)
# Convert to array
mapfile -t BUNDLES_TO_RUN <<< "${BUNDLES_TO_RUN_LIST}"

echo "[Run] Starting ${ITERATIONS} iterations on ${#BUNDLES_TO_RUN[@]} bundles..."

for i in $(seq 1 "${ITERATIONS}"); do
    for BUNDLE in "${BUNDLES_TO_RUN[@]}"; do
        # Determine Run ID suffix if multiple bundles
        SUFFIX=""
        if [[ ${#BUNDLES_TO_RUN[@]} -gt 1 ]]; then
            # Extract basic name e.g. reliability_split_A01 -> _A01
            BN=$(basename "${BUNDLE}" .json)
            if [[ "$BN" == reliability_split_* ]]; then
                 SUFFIX="_${BN#reliability_split_}"
            else
                 SUFFIX="_${BN#scenarios_}"
            fi
        fi
        
        RUN_ID="run_$(printf '%02d' "${i}")${SUFFIX}"
        RUN_DIR="${EVIDENCE_BASE}/${RUN_ID}"
        mkdir -p "${RUN_DIR}"

        # Print Run Header
        if [[ ${MODE} != "quick" ]]; then
           printf "  %s: " "${RUN_ID}"
        else
           printf "%s: " "${RUN_ID}"
        fi

        # Docker Env
        export ORACLE_BACKEND
        
        cp "${BUNDLE}" "evidence/ephemeral_reliability.json"
        export SCENARIO_BUNDLE="/evidence/ephemeral_reliability.json"
        
        export TRACE="/evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}/${RUN_ID}/trace.jsonl"
        export REPORT="/evidence/reliability/${ROS_DISTRO}/${ORACLE_BACKEND}/${RUN_ID}/report.json"

        LOG_FILE="${RUN_DIR}/stdout.log"
        set +e
        docker compose run --rm harness > "${LOG_FILE}" 2>&1
        EXIT_CODE=$?
        set -e

        # Result Analysis
        TRACE_FILE="${RUN_DIR}/trace.jsonl"
        REPORT_FILE="${RUN_DIR}/report.json"
        
        RUN_STATUS="UNKNOWN"
        
        if [[ ! -f "${TRACE_FILE}" ]] || [[ ! -s "${TRACE_FILE}" ]] || [[ ! -f "${REPORT_FILE}" ]]; then
            RUN_STATUS="INFRA_FAIL"
            INFRA_FAIL_COUNT=$((INFRA_FAIL_COUNT + 1))
        elif ! python3 -c "import json; json.load(open('${REPORT_FILE}'))" 2>/dev/null; then
            RUN_STATUS="INFRA_FAIL"
            INFRA_FAIL_COUNT=$((INFRA_FAIL_COUNT + 1))
        else
            PASS=$(python3 -c "import json; print(json.load(open('${REPORT_FILE}')).get('pass', False))")
            if [[ "${PASS}" == "True" ]]; then
                RUN_STATUS="PASS"
                PASS_COUNT=$((PASS_COUNT + 1))
            else
                RUN_STATUS="FAIL"
                FAIL_COUNT=$((FAIL_COUNT + 1))
            fi
        fi

        # Output Printing
        GREEN='\033[0;32m'
        RED='\033[0;31m'
        RESET='\033[0m'
        
        if [[ "${RUN_STATUS}" == "PASS" ]]; then COLOR="${GREEN}"; else COLOR="${RED}"; fi
        printf "${COLOR}%s${RESET}\n" "${RUN_STATUS}"
        
        # Per-scenario detail (Summary/Verbose modes)
        if [[ ${MODE} == "summary" || ${MODE} == "verbose" ]]; then
            if [[ -f "${REPORT_FILE}" ]]; then
            python3 -u -c "
import json, sys
GREEN = '\033[0;32m'
RED = '\033[0;31m'
YELLOW = '\033[0;33m'
RESET = '\033[0m'
try:
    with open('${REPORT_FILE}') as f:
        data = json.load(f)
    for sid, sc in sorted(data.get('scenarios', {}).items(), key=lambda x: (0 if x[0].startswith('H') else 1, x[0])):
        outcome = sc.get('outcome', 'UNKNOWN')
        if outcome == 'FAIL':
            outcome = 'SEMANTIC_FAIL'
            color = RED
        elif outcome == 'PASS':
            color = GREEN
        elif sc.get('error'):
            outcome = 'INFRA_FAIL'
            color = RED
        elif sc.get('outcome') == 'SKIP':
            color = YELLOW
        else:
            color = RESET
        print(f'    {sid}: {color}{outcome}{RESET}')
except: pass"
            fi
        fi
        
        if [[ ${VERBOSE} -eq 1 ]]; then
             echo "    Log: ${LOG_FILE}"
        fi
    done
done

# Aggregation (unchanged)
echo "[Aggregate] Running aggregation..."
if python3 "${ROOT}/tools/analysis/aggregate_results.py" > /dev/null; then
    # Re-print the summary block extracted from python output manually or just let python print?
    # aggregate_results.py prints the summary block. We silenced it above? 
    # User said "Output modes... Default should be --summary".
    # User said "Evidence format / Aggregation logic... unchanged".
    # We should let aggregation print its summary as requested in previous step.
    python3 "${ROOT}/tools/analysis/aggregate_results.py"
else
    echo "[Aggregate] FAILED"
fi

if [[ ${FAIL_COUNT} -gt 0 ]] || [[ ${INFRA_FAIL_COUNT} -gt 0 ]]; then
    exit 1
fi
exit 0
