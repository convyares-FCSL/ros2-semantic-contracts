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
# 2. Select peer binary based on bundle content.
#    Scan all send_goal ops; use the FIRST one's payload.mode.
#    "reject" → reject_always.  Anything else → sequence.
# ---------------------------------------------------------------------------
MODE=$(python3 -c "
import json, sys
bundle = json.load(open('${BUNDLE}'))
for sid, sc in sorted(bundle.get('scenarios', {}).items()):
    for op in sc.get('ops', []):
        if op.get('op') == 'send_goal':
            print(op.get('payload', {}).get('mode', 'accept'))
            sys.exit(0)
print('accept')
")

if [[ "${MODE}" == "reject" ]]; then
    PEER_BIN="${PEER_DIR}/peer_reject_always"
else
    PEER_BIN="${PEER_DIR}/peer_sequence"
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
