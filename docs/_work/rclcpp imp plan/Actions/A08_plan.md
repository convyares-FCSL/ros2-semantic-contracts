 A08 Implementation Plan: Cancellation Intent Observable

 Context

 Problem: The ROS 2 semantic contracts harness needed to validate SPEC_AC06 (Core), which states that cancellation intent must be explicitly observable through:
 1. The cancel service interaction (request + response)
 2. The status topic transition to STATUS_CANCELING

 Why this matters: Cancellation in ROS 2 Actions is a request, not a command. The client cannot force an outcome—it can only signal intent. The core semantic contract requires that this
 intent be observable in the trace, separate from the eventual terminal outcome (which may be SUCCEEDED, ABORTED, or CANCELED depending on timing and policy).

 Prior state: Before A08 implementation:
 - The harness had no cancel_goal operation
 - The action_server actor immediately processed outcomes with no delay
 - Cancel requests would often arrive after goal completion, making STATUS_CANCELING unobservable
 - The oracle core didn't recognize cancel_request or cancel_response event types

 What A08 validates:
 - Observable evidence that a cancel was requested (cancel_request event)
 - Observable evidence that the server acknowledged the cancel (cancel_response with accepted boolean)
 - Observable evidence of the state transition (STATUS_CANCELING status event)

 What A08 does NOT validate:
 - Terminal outcome after cancellation (that's A09 - cancel resolution policy)
 - Cancel rejection scenarios (future work)
 - Multi-goal cancellation (future work)

 ---
 Implementation Approach

 1. Backend Operation: cancel_goal

 Files Modified:
 - harness/backends/backend_prod/src/backend.hpp (lines 47-49)
 - harness/backends/backend_prod/src/ops/actions.cpp (lines 181-250)
 - harness/backends/backend_prod/src/backend.cpp (lines 13, 43-44)

 Implementation:

 Add Goal Handle Storage (backend.hpp)

 // NEW: Goal handle storage (for cancel operations)
 // goal_id (string) -> goal_handle (shared_ptr)
 std::map<std::string, rclcpp_action::ClientGoalHandle<FibAction>::SharedPtr> goal_handles;

 This map stores goal handles indexed by scenario goal_id string (e.g., "g_cancel"). Thread-safe access via st.map_mutex.

 Store Goal Handles in send_goal (actions.cpp:159)

 When a goal is accepted, store the handle:
 if (accepted) {
     std::lock_guard<std::mutex> lock(st.map_mutex);
     st.uuid_map[uuid_vec] = goal_id_str;
     st.uuid_scenario_map[uuid_vec] = scenario_id;
     st.goal_handles[goal_id_str] = goal_handle;  // NEW
 }

 Implement exec_cancel_goal (actions.cpp:185-250)

 Step-by-step flow:

 1. Validate input: Check that op.goal_id is present, throw BundleError if missing
 2. Drain status buffer: Clear any pending status events before cancel
 3. Lookup goal handle: Thread-safe lookup in st.goal_handles map
 4. Emit cancel_request: Observable event indicating cancel intent
 w.emit({
     {"type", "cancel_request"},
     {"scenario_id", scenario_id},
     {"goal_id", goal_id_str}
 });
 5. Call async_cancel_goal: Real ROS 2 action client API
 auto future_cancel = st.client->async_cancel_goal(goal_handle);
 6. Wait for response: Block for up to 2 seconds
 7. Map return code:
   - return_code == 0 (ERROR_NONE) → accepted: true
   - Other codes → accepted: false
 8. Emit cancel_response: Observable event with both boolean and numeric response
 w.emit({
     {"type", "cancel_response"},
     {"scenario_id", scenario_id},
     {"goal_id", goal_id_str},
     {"accepted", accepted},
     {"return_code", return_code}
 });
 9. Drain status buffer again: Capture STATUS_CANCELING events that arrived during RPC

 Add Dispatch (backend.cpp:43-44)

 } else if (op.op == "cancel_goal") {
     ops::exec_cancel_goal(w, st, scenario_id, op);

 ---
 2. Actor Enhancement: Cancel-Aware action_server

 File Modified:
 - harness/backends/backend_prod/actors/action_server.cpp (lines 4-11, 91, 113-151)

 Problem solved: Without modification, the action_server would immediately execute outcomes, leaving no window for cancel requests to arrive and be observed.

 Implementation:

 Add execute_delay_ms Parameter (line 91)

 node->declare_parameter("execute_delay_ms", 0);  // Default 0 for backward compatibility

 Modify Execute Callback to Poll for Cancellation (lines 113-151)

 Before delay processing:
 int64_t delay_ms = node->get_parameter("execute_delay_ms").as_int();

 std::thread([goal_handle, sequence, delay_ms]() {
     auto result = std::make_shared<FibAction::Result>();
     result->sequence = {1, 1, 2, 3, 5};

     // NEW: Execute delay with cancel polling
     const int64_t check_interval_ms = 10;
     int64_t elapsed_ms = 0;
     while (elapsed_ms < delay_ms) {
         if (goal_handle->is_canceling()) {
             goal_handle->canceled(result);
             return;  // Exit immediately
         }
         std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
         elapsed_ms += check_interval_ms;
     }

     // Final check after delay
     if (goal_handle->is_canceling()) {
         goal_handle->canceled(result);
         return;
     }

     // Process outcome_sequence only if not canceled
     for (const auto& outcome : sequence) {
         // ... existing outcome processing ...
     }
 }).detach();

 Key properties:
 - 10ms check interval: Polls is_canceling() every 10ms during delay
 - Real rclcpp API: Uses goal_handle->is_canceling() and goal_handle->canceled()
 - Early exit: Returns immediately if cancel detected, preventing outcome processing
 - No simulation: The ROS 2 infrastructure handles status transitions (STATUS_CANCELING)
 - Backward compatible: execute_delay_ms: 0 behaves like the original implementation

 ---
 3. Oracle Core: Event Type Registration

 File Modified:
 - harness/core/src/io.rs (lines 33-34)

 Change:
 Add cancel_request and cancel_response to the CORE_TYPES whitelist:

 const CORE_TYPES: &[&str] = &[
     // ... existing types ...
     "status",
     "result",
     "cancel_request",   // NEW
     "cancel_response",  // NEW
     "terminal_set_attempt",
     // ... rest ...
 ];

 This allows the oracle to accept these event types during trace validation. Without this, the oracle would reject traces containing cancel events as "unknown event type."

 ---
 4. Scenario Definition

 File Modified:
 - harness/scenarios/scenarios_A.json (lines 258-330)

 Scenario Structure:

 {
     "A08_cancel_intent_observable": {
         "spec_id": "A08",
         "title": "Cancellation intent observable",
         "layer": "Core",
         "requires": ["actions.basic"],
         "notes": "Validates SPEC_AC06: cancellation intent must be observable via cancel service and status topic. Observable assertions: (1) cancel service interaction (cancel_request
 + cancel_response), (2) STATUS_CANCELING transition. Does NOT assert terminal outcome after Canceling (that's A09).",
         "boundary": {
             "sut": "client",
             "peer": "external",
             "peer_rule": "Peer is a generic scenario-agnostic actor. Backend configures its behavior profile via ROS parameters through the start_actor op, per boundaries.md section 3."
         },
         "ops": [
             {
                 "op": "start_actor",
                 "actor": "action_server",
                 "params": {
                     "goal_response": "accept",
                     "outcome_sequence": "succeed",
                     "execute_delay_ms": 200
                 }
             },
             {
                 "op": "send_goal",
                 "goal_id": "g_cancel"
             },
             {
                 "op": "wait",
                 "ms": 50
             },
             {
                 "op": "cancel_goal",
                 "goal_id": "g_cancel"
             },
             {
                 "op": "wait",
                 "ms": 200
             },
             {
                 "op": "stop_actor",
                 "actor": "action_server"
             }
         ],
         "expects": [
             {
                 "check": "custom",
                 "params": {
                     "must_observe": [
                         {
                             "type": "goal_send_decision",
                             "goal_id": "g_cancel",
                             "accepted": true
                         },
                         {
                             "type": "cancel_request",
                             "goal_id": "g_cancel"
                         },
                         {
                             "type": "cancel_response",
                             "goal_id": "g_cancel",
                             "accepted": true
                         },
                         {
                             "type": "status",
                             "goal_id": "g_cancel",
                             "status": "STATUS_CANCELING"
                         }
                     ]
                 }
             }
         ]
     }
 }

 Timing rationale:
 - execute_delay_ms: 200 - Actor stays in EXECUTING for 200ms
 - wait 50ms before cancel - Let goal start executing
 - cancel_goal - Send cancel during the delay window
 - wait 200ms after cancel - Allow status events to propagate and be observed

 ---
 5. Documentation

 File Modified:
 - docs/provenance/scenario/scenario_execution_matrix.md (line 44)

 Change:
 | **A08** | A08_cancel_intent_observable       | N/A  | N/A  | PASS | N/A   | Cancel service interaction + STATUS_CANCELING observable |

 Updates the coverage matrix to reflect that backend_prod (PROD column) now passes A08.

 ---
 Critical Files
 ┌────────────────────────────────────────────────────────┬───────────────────────┬─────────────────────────────────────────────────────────────────────────────────┐
 │                          File                          │        Purpose        │                                   Key Changes                                   │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/backend.hpp          │ State storage         │ Added goal_handles map (line 48)                                                │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/ops/actions.cpp      │ Cancel operation      │ Implemented exec_cancel_goal (lines 185-250), updated exec_send_goal (line 159) │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/backend.cpp          │ Operation dispatch    │ Added cancel_goal dispatch (lines 13, 43-44)                                    │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/actors/action_server.cpp │ Cancel-aware actor    │ Added execute_delay_ms param and cancel polling loop (lines 91, 113-151)        │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ harness/core/src/io.rs                                 │ Event type validation │ Added cancel_request, cancel_response to CORE_TYPES (lines 33-34)               │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ harness/scenarios/scenarios_A.json                     │ Scenario definition   │ Populated A08 with ops and expects (lines 258-330)                              │
 ├────────────────────────────────────────────────────────┼───────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ docs/provenance/scenario/scenario_execution_matrix.md  │ Coverage tracking     │ Updated A08 to PASS (line 44)                                                   │
 └────────────────────────────────────────────────────────┴───────────────────────┴─────────────────────────────────────────────────────────────────────────────────┘
 ---
 Reusable Functions & Patterns

 Existing patterns leveraged:
 1. Status subscription + buffering (actions.cpp:27-94) - Reused for capturing STATUS_CANCELING
 2. Status drain helper (status_drain.hpp:12-18) - Called before/after cancel_goal
 3. Goal handle storage pattern - Extended existing uuid_map pattern with goal_handles map
 4. Custom check with must_observe - Reused existing validation pattern from A01/A02/A06
 5. Actor parameter configuration - Extended existing start_actor ROS parameter pattern

 New patterns introduced:
 1. Request/response event pairs - cancel_request → cancel_response (mirrors param operations)
 2. Cancel-aware execute loop - Polling is_canceling() with configurable delay (reusable for future scenarios)

 ---
 Verification

 Local Test

 ORACLE_BACKEND=prod bash harness/scripts/run_harness.sh \
     harness/scenarios/scenarios_A.json \
     /tmp/a08_trace.jsonl \
     /tmp/a08_report.json

 Expected output:
 PASS A08_cancel_intent_observable (A08) — Cancellation intent observable

 Trace Inspection

 cat /tmp/a08_trace.jsonl | jq 'select(.scenario_id == "A08_cancel_intent_observable") |
     select(.type | IN("cancel_request", "cancel_response", "status")) |
     {seq: .sequence, type, goal_id, status, accepted, return_code}'

 Expected sequence:
 {"seq": 58, "type": "status", "goal_id": "g_cancel", "status": "STATUS_EXECUTING"}
 {"seq": 61, "type": "cancel_request", "goal_id": "g_cancel"}
 {"seq": 62, "type": "cancel_response", "goal_id": "g_cancel", "accepted": true, "return_code": 0}
 {"seq": 63, "type": "status", "goal_id": "g_cancel", "status": "STATUS_CANCELING"}
 {"seq": 67, "type": "status", "goal_id": "g_cancel", "status": "STATUS_CANCELED"}

 Key validation points:
 - ✓ cancel_request appears before cancel_response (causality)
 - ✓ cancel_response has accepted: true and return_code: 0
 - ✓ STATUS_CANCELING appears after cancel response (observable transition)
 - ✓ Sequence numbers are monotonic (ordering preserved)

 Report Validation

 cat /tmp/a08_report.json | jq '.scenarios.A08_cancel_intent_observable'

 Expected:
 {
   "outcome": "PASS",
   "details": {
     "observed_end": true,
     "observed_start": true
   },
   "layer": "Core",
   "notes": "Validates SPEC_AC06...",
   "spec_id": "A08",
   "title": "Cancellation intent observable"
 }

 Determinism Validation (5-run Docker protocol)

 for i in {1..5}; do
     docker compose build
     docker compose up --abort-on-container-exit
     cp evidence/report.json evidence/a08_run_$i.json
 done

 # Compare all reports - should be identical
 for i in {2..5}; do
     diff <(jq -S .scenarios.A08_cancel_intent_observable evidence/a08_run_1.json) \
          <(jq -S .scenarios.A08_cancel_intent_observable evidence/a08_run_${i}.json)
 done

 Success criteria:
 - 5/5 runs show A08 outcome = "PASS"
 - No differences between runs (deterministic)
 - No regressions in other scenarios (A01, A02, A06 still pass)

 ---
 Design Rationale

 Why Not Use Generic op_start/op_end?

 Decision: Use explicit cancel_request/cancel_response events instead of generic operation lifecycle events.

 Rationale:
 - Matches existing domain event pattern (goal_send_decision, param_set_request/response)
 - Trace schema already includes these types (lines 38-39)
 - More explicit signal for oracle validation
 - Aligns with observable evidence philosophy (domain semantics, not implementation artifacts)

 Why Polling Instead of Callback?

 Decision: Poll is_canceling() every 10ms during execute_delay instead of using a cancel callback.

 Rationale:
 - The actor already has a cancel callback (line 101) that accepts all cancels
 - The execute thread needs to check if cancellation was accepted
 - rclcpp_action design: cancel callback returns acceptance decision, execute thread checks is_canceling() to react
 - Polling with 10ms interval is lightweight and deterministic
 - Creates predictable test conditions (200ms delay guarantees 20 checks)

 Why 200ms Delay?

 Decision: Use execute_delay_ms: 200 for A08.

 Rationale:
 - Long enough to guarantee goal reaches EXECUTING before cancel arrives (50ms + network latency < 200ms)
 - Short enough to keep test runtime reasonable (total A08 runtime ~500ms)
 - Provides ~15-20 polling opportunities (200ms / 10ms) for high determinism
 - Can be adjusted per-scenario if needed (different scenarios can use different delays)

 Why Drain Status Buffer Twice?

 Decision: Call drain_status_buffer() before AND after async_cancel_goal().

 Rationale:
 - Before: Clear any stale status events from previous operations (clean slate)
 - After: Capture STATUS_CANCELING events that arrived during the cancel RPC
 - Status subscription runs asynchronously on executor thread
 - Buffer ensures events aren't lost or misordered relative to cancel_request/cancel_response
 - Thread-safe via status_buffer_mutex

 ---
 Risk Mitigation

 Risk: Cancel arrives before EXECUTING

 Symptom: Goal goes ACCEPTED → CANCELED (skips EXECUTING/CANCELING)
 Mitigation: 50ms wait between send_goal and cancel_goal + 200ms execute_delay
 Fallback: If still racy, increase wait or add explicit wait_for_status check

 Risk: STATUS_CANCELING is transient (missed)

 Symptom: Status buffer doesn't capture brief CANCELING transition
 Mitigation:
 - Status subscription with reliable QoS
 - Buffer-based collection (not immediate emit)
 - 200ms wait after cancel ensures buffer is drained
 Fallback: Increase post-cancel wait if events are missed

 Risk: Non-deterministic terminal outcome

 Symptom: Sometimes CANCELED, sometimes SUCCEEDED/ABORTED
 Mitigation: A08 does NOT assert terminal outcome (out of scope)
 Note: Terminal outcome validation is A09 responsibility

 Risk: Goal handle not found

 Symptom: cancel_goal throws BundleError "goal_id not found"
 Mitigation:
 - Defensive check in exec_cancel_goal (lines 195-207)
 - Clear error message identifies missing goal_id
 - Unit-testable with invalid goal_id
 Fallback: Ensure send_goal stores handle before cancel_goal is called

 ---
 Summary

 A08 validates SPEC_AC06 by implementing:
 1. Backend operation - Real ROS 2 cancel API with observable request/response events
 2. Actor enhancement - Cancel-aware execute loop with configurable delay for determinism
 3. Scenario definition - Timing-controlled operation sequence with explicit expectations
 4. Schema extension - Recognition of cancel event types in oracle core

 The implementation provides deterministic, observable validation of cancellation intent without simulation or timing superstitions. Total implementation: ~150 lines of C++ (backend +
 actor) + ~50 lines of JSON (scenario) + 2 lines of Rust (event types).

 Status: ✅ Implemented and passing (4/17 action scenarios now validated)