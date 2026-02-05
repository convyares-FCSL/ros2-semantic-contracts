#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Peer launcher for backend_prod scenarios.
#
# Responsibilities (in order):
#   1. Derive RUN_ID (from env or generate).
#   2. Scan the bundle for the first send_goal op's payload.mode.
#   3. Select the correct peer binary (reject_always or sequence).
#   4. Launch the peer in the background.
#   5. Wait for the sentinel file (/tmp/peer_ready_<RUN_ID>) with a timeout.
#   6. Invoke oracle_core (the arguments are forwarded from the caller).
#   7. Capture oracle_core's exit code.
#   8. Send SIGTERM to the peer and wait for it.
#   9. Clean up the sentinel (belt-and-suspenders; peer also removes it).
#  10. Exit with oracle_core's exit code.
#
# Usage (internal — called by run_harness.sh):
#   launch_peer.sh <bundle_path> <trace_path> <report_path> <backend_bin> <core_bin>
#
# Environment:
#   BACKEND_PROD_RUN_ID   – run identifier.  Generated if absent.
#   PEER_SENTINEL_TIMEOUT – seconds before aborting (default 5).
# ---------------------------------------------------------------------------
set -euo pipefail

BUNDLE="$1"
TRACE="$2"
REPORT="$3"
BACKEND_BIN="$4"
CORE_BIN="$5"

# Locate peer binaries relative to backend_prod binary location.
PEER_DIR="$(dirname "${BACKEND_BIN}")"

# ---------------------------------------------------------------------------
# 1. RUN_ID
# ---------------------------------------------------------------------------
export BACKEND_PROD_RUN_ID="${BACKEND_PROD_RUN_ID:-run_prod}"
RUN_ID="${BACKEND_PROD_RUN_ID}"
SENTINEL="/tmp/peer_ready_${RUN_ID}"
TIMEOUT="${PEER_SENTINEL_TIMEOUT:-5}"

# ---------------------------------------------------------------------------
# 2. Select peer binary based on TARGET_SCENARIO_ID or strict bundle scan.
# ---------------------------------------------------------------------------

# Detect start_actor usage. If present, we bypass external peer launch.
HAS_ACTOR=$(python3 -c "
import json, sys
try:
    bundle = json.load(open('${BUNDLE}'))
    found = False
    for s in bundle.get('scenarios', {}).values():
        for op in s.get('ops', []):
            if op.get('op') == 'start_actor':
                found = True
                break
        if found: break
    print('yes' if found else 'no')
except:
    print('no')
")

if [[ "${HAS_ACTOR}" == "yes" ]]; then
    # Actor Model: Backend manages the actor lifecycle. No external peer needed.
    echo "DEBUG: launch_peer detected start_actor op — skipping external peer launch" >&2
    
    # Just run the core (wrapper) -> backend
    "${CORE_BIN}" "${BUNDLE}" "${TRACE}" "${REPORT}" --backend "${BACKEND_BIN}"
    exit $?
fi

TARGET_ID="${TARGET_SCENARIO_ID:-}"

if [[ -n "${TARGET_ID}" ]]; then
    # Explicit selection from environment
    if [[ "${TARGET_ID}" == "A01"* ]]; then
        MODE="reject"
    elif [[ "${TARGET_ID}" == "A02"* ]]; then
        MODE="succeed_abort"
    else
        MODE="sequence"
    fi
else
    # Auto-detection from bundle (Strict Mode)
    # Fail fast if multiple Axx scenarios are detected without explicit target.
    # Legacy Cleanup: The old 'reject' and 'succeed_abort' peers are gone.
    # If we are here, we are likely running an H-series scenario which defaults to 'sequence'.
    # If A01/A02 are detected here (which shouldn't happen if they use start_actor), we warn.

    DETECTED_MODES=$(python3 -c "
import json, sys
try:
    bundle = json.load(open('${BUNDLE}'))
    modes = set()
    for sid in bundle.get('scenarios', {}):
        if sid.startswith('H'): modes.add('sequence')
        elif sid.startswith('A'): modes.add('actor_should_handle')
        else: modes.add('sequence')

    if 'actor_should_handle' in modes:
        print('ACTOR_MISMATCH')
    else:
        print('sequence')
except:
    print('sequence')
")

    if [[ "${DETECTED_MODES}" == "ACTOR_MISMATCH" ]]; then
        echo "ERROR: launch_peer invoked for A-series scenario. These should use start_actor and bypass this script." >&2
        exit 4
    fi
    MODE="sequence"
    echo "DEBUG: launch_peer detected mode: ${MODE}" >&2
fi

if [[ "${MODE}" == "sequence" ]]; then
    PEER_BIN="${PEER_DIR}/sequence_actor"
else
    # Fallback/Error
    echo "ERROR: Unknown peer mode '${MODE}' requested." >&2
    exit 4
fi

if [[ ! -x "${PEER_BIN}" ]]; then
    echo "ERROR: peer binary not found or not executable: ${PEER_BIN}" >&2
    exit 4
fi

# ---------------------------------------------------------------------------
# 3. Remove any stale sentinel left by a previously killed run with the same
#    RUN_ID.  Safe: if we are here, no live peer for this RUN_ID exists
#    (the launcher is the sole entry-point that starts one).  The atomic
#    O_CREAT|O_EXCL in the peer binary still catches same-run races.
# ---------------------------------------------------------------------------
rm -f "${SENTINEL}"

# ---------------------------------------------------------------------------
# 4. Launch peer in background.
# ---------------------------------------------------------------------------
"${PEER_BIN}" &
PEER_PID=$!

# ---------------------------------------------------------------------------
# 5. Wait for sentinel with timeout.
# ---------------------------------------------------------------------------
TIMEOUT_TICKS=$(( TIMEOUT * 10 ))   # each tick = 100 ms
ELAPSED=0
while [[ ! -e "${SENTINEL}" ]]; do
    if [[ ${ELAPSED} -ge ${TIMEOUT_TICKS} ]]; then
        echo "ERROR: sentinel ${SENTINEL} did not appear within ${TIMEOUT}s — peer may have crashed" >&2
        kill "${PEER_PID}" 2>/dev/null || true
        wait "${PEER_PID}" 2>/dev/null || true
        exit 4
    fi
    sleep 0.1
    ELAPSED=$(( ELAPSED + 1 ))
done

# ---------------------------------------------------------------------------
# 6-7. Invoke oracle_core; capture exit code.
# ---------------------------------------------------------------------------
CORE_EXIT=0
"${CORE_BIN}" "${BUNDLE}" "${TRACE}" "${REPORT}" --backend "${BACKEND_BIN}" || CORE_EXIT=$?

# ---------------------------------------------------------------------------
# 8. Graceful peer shutdown.
# ---------------------------------------------------------------------------
kill "${PEER_PID}" 2>/dev/null || true
wait "${PEER_PID}" 2>/dev/null || true

# ---------------------------------------------------------------------------
# 9. Belt-and-suspenders sentinel cleanup.
# ---------------------------------------------------------------------------
rm -f "${SENTINEL}"

# ---------------------------------------------------------------------------
# 10. Propagate oracle_core's exit code.
# ---------------------------------------------------------------------------
exit ${CORE_EXIT}
