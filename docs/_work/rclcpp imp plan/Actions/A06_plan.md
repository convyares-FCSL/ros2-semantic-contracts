 Implementation Plan: A06_status_updates_monotonic

 Context

 A06 validates SPEC_AC05 (Core): Status updates for a single goal must be monotonically ordered and follow the valid transition graph per SPEC_AC04 (action_core.md lines 84-110). This
 ensures:
 - Status sequences like ACCEPTED → EXECUTING → SUCCEEDED are observable
 - No backward transitions (e.g., EXECUTING → ACCEPTED is forbidden)
 - Ordering is determined by sequence numbers, not wall-clock time

 Current State:
 - A06 scenario is a stub in scenarios_A.json (empty ops/expects)
 - Backend only emits goal_send_decision and result events
 - No status observation infrastructure exists - intermediate states (ACCEPTED, EXECUTING) are invisible
 - rclcpp_action publishes status updates to _action/status topic, but backend doesn't subscribe
 - Oracle has no checker to validate status monotonicity - existing custom checker doesn't enforce ordering

 Why This Change:
 A06 validates SPEC_AC05 (Core), requiring both:
 1. Backend changes - Subscribe to status topic and emit status events
 2. Oracle changes - Add status_monotonic checker to validate transitions and detect regressions

 This is scenario #6 in Track 1 (PLAN.md line 42), required for the 8/10 switch gate. It unlocks A07 (ordering) and A08 (cancellation) which depend on status observation infrastructure.

 ---
 Implementation Strategy

 Core Insight

 rclcpp SendGoalOptions does NOT have a status_callback - status updates are published to the <action>/_action/status topic. We must subscribe to this topic to observe intermediate
 states.

 Three-Part Implementation

 1. Backend Infrastructure (actions.cpp + backend.hpp)
 - Subscribe to test_action/_action/status topic (create once per backend)
 - Emit status trace events with goal_id, "status" field, auto-incrementing sequence
 - Track (UUID → scenario_id) mapping for callback context
 - Critical: Field name is "status" (not "state") per backend_stub:375 and backend_ros:556

 2. Oracle Checker (eval.rs)
 - Add status_monotonic checker type to validate:
   - Status events sorted by sequence field
   - Each transition follows valid state graph (per SPEC_AC04)
   - No regressions (e.g., EXECUTING → ACCEPTED)
   - Terminal states are immutable

 3. Scenario Definition (scenarios_A.json)
 - Populate A06 with standard ops pattern: start_actor → send_goal → wait → stop_actor
 - Use status_monotonic checker to validate complete monotonicity
 - Use custom checker to validate goal acceptance and result

 4. Validation Strategy
 - 300ms wait ensures status messages propagate before scenario ends
 - Status topic publishing is deterministic in rclcpp_action
 - Sequence field guarantees ordering (line 30 in trace.cpp: seq_++ is mutex-protected)

 ---
 Critical Files & Changes

 1. /harness/backends/backend_prod/src/backend.hpp (lines 24-56)

 Add include at top:

 #include <deque>

 Add status subscription and scenario tracking to BackendState:

 struct BackendState {
     // ROS 2 Infrastructure
     rclcpp::Node::SharedPtr node;
     rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
     std::thread spinner_thread;

     // Action Logic
     rclcpp_action::Client<FibAction>::SharedPtr client;

     // NEW: Status subscription (shared across scenarios)
     rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub;

     // UUID to goal_id mapping
     std::mutex map_mutex;
     std::map<std::vector<uint8_t>, std::string> uuid_map;

     // NEW: UUID to scenario_id mapping (for status callback context)
     std::map<std::vector<uint8_t>, std::string> uuid_scenario_map;

     // NEW: Thread-safe buffer for pending status events
     // Status subscription appends here; ops drain and emit via TraceWriter
     std::mutex status_buffer_mutex;
     std::deque<json> pending_status_events;

     // goal_id → terminal_status
     std::unordered_map<std::string, std::string> terminal;
     std::unordered_map<std::string, uint32_t> terminal_attempts;

     // Actor name → PID
     std::map<std::string, pid_t> pid_map;

     ~BackendState() {
         if (executor) executor->cancel();
         if (spinner_thread.joinable()) spinner_thread.join();
     }
 };

 Rationale: Status subscription is created once and shared. The callback needs scenario_id to emit events, which we track via uuid_scenario_map.

 ---
 2. /harness/backends/backend_prod/src/ops/actions.cpp

 Add includes (after line 4):

 #include <action_msgs/msg/goal_status.hpp>
 #include <action_msgs/msg/goal_status_array.hpp>

 Add helper function (before exec_send_goal):

 // Helper: Drain pending status events and emit via TraceWriter
 // SAFETY: Only call when TraceWriter& w is valid (during op execution)
 static void drain_status_buffer(TraceWriter& w, BackendState& st) {
     std::lock_guard<std::mutex> lock(st.status_buffer_mutex);
     for (const auto& ev : st.pending_status_events) {
         w.emit(ev);  // TraceWriter adds sequence, t_ns, version, run_id
     }
     st.pending_status_events.clear();
 }

 Modify exec_send_goal (lines 12-94):

 void exec_send_goal(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
     if (!op.goal_id) {
         throw BundleError("send_goal: missing goal_id");
     }
     const std::string& goal_id_str = *op.goal_id;

     if (!st.client) {
         throw SystemError("Action client not initialized");
     }

     // Create status subscription if not already created
     // SAFETY: Callback does NOT capture TraceWriter& w - only appends to buffer
     if (!st.status_sub) {
         st.status_sub = st.node->create_subscription<action_msgs::msg::GoalStatusArray>(
             "test_action/_action/status",
             10,
             [&st](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                 // CRITICAL: Do NOT capture TraceWriter& w here (dangling reference risk)
                 // Instead, append to buffer and let ops drain during execution

                 std::lock_guard<std::mutex> map_lock(st.map_mutex);
                 for (const auto& status_item : msg->status_list) {
                     // Convert UUID to vector<uint8_t> for map lookup
                     std::vector<uint8_t> uuid_vec(
                         status_item.goal_info.goal_id.uuid.begin(),
                         status_item.goal_info.goal_id.uuid.end()
                     );

                     // Check if this goal belongs to our scenarios
                     auto goal_it = st.uuid_map.find(uuid_vec);
                     auto scenario_it = st.uuid_scenario_map.find(uuid_vec);

                     if (goal_it != st.uuid_map.end() && scenario_it != st.uuid_scenario_map.end()) {
                         const std::string& goal_id = goal_it->second;
                         const std::string& scenario_id = scenario_it->second;

                         // Map action_msgs::GoalStatus to status string
                         std::string status_str;
                         switch (status_item.status) {
                             case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
                                 status_str = "STATUS_ACCEPTED"; break;
                             case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                                 status_str = "STATUS_EXECUTING"; break;
                             case action_msgs::msg::GoalStatus::STATUS_CANCELING:
                                 status_str = "STATUS_CANCELING"; break;
                             case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                                 status_str = "STATUS_SUCCEEDED"; break;
                             case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                                 status_str = "STATUS_CANCELED"; break;
                             case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                                 status_str = "STATUS_ABORTED"; break;
                             default:
                                 status_str = "STATUS_UNKNOWN"; break;
                         }

                         // Append to buffer (thread-safe)
                         {
                             std::lock_guard<std::mutex> buffer_lock(st.status_buffer_mutex);
                             st.pending_status_events.push_back({
                                 {"type", "status"},
                                 {"scenario_id", scenario_id},
                                 {"goal_id", goal_id},
                                 {"status", status_str}
                             });
                         }
                     }
                 }
             }
         );
     }

     // Drain any pending status events from previous operations
     drain_status_buffer(w, st);

     // [EXISTING CODE: wait_for_action_server, send goal, result callback...]

     // After goal acceptance, register UUID mappings (modify existing code around line 74):
     if (accepted) {
         auto uuid = goal_handle->get_goal_id();
         std::vector<uint8_t> uuid_vec(uuid.begin(), uuid.end());
         {
             std::lock_guard<std::mutex> lock(st.map_mutex);
             st.uuid_map[uuid_vec] = goal_id_str;
             st.uuid_scenario_map[uuid_vec] = scenario_id;
         }
     }

     // Emit goal_send_decision
     w.emit({
         {"type", "goal_send_decision"},
         {"scenario_id", scenario_id},
         {"goal_id", goal_id_str},
         {"accepted", accepted}
     });

     // Drain status events that arrived during send_goal
     drain_status_buffer(w, st);
 }

 Key Safety Notes:
 - Callback does NOT capture TraceWriter& w - only appends to buffer
 - pending_status_events is thread-safe (mutex-protected)
 - drain_status_buffer() is called at safe points where w is valid:
   - Start of send_goal (drain old events)
   - End of send_goal (drain events from goal acceptance)
   - End of wait (drain events that arrived during wait)
 - TraceWriter automatically adds sequence, t_ns, version, run_id

 ---
 3. /harness/backends/backend_prod/src/ops/generic.cpp

 Modify exec_wait to drain status buffer after wait:

 void exec_wait(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
     // [EXISTING WAIT LOGIC - get ms, sleep]
     uint64_t ms = op.ms.value_or(0);
     std::this_thread::sleep_for(std::chrono::milliseconds(ms));

     // NEW: Drain status buffer after wait (status messages arrive during sleep)
     drain_status_buffer(w, st);
 }

 Rationale: Status messages arrive asynchronously during wait periods. After the wait completes, we drain the buffer while TraceWriter& w is still valid, ensuring status events appear in
  the trace with proper sequence ordering.

 ---
 4. /harness/core/src/eval.rs

 Add status_monotonic checker (after line 190, before describe_unknown_consistent):

 "status_monotonic" => {
     let params = ex.params.as_ref().unwrap_or(&Value::Null);

     let goal_id = params.get("goal_id").and_then(|v| v.as_str()).ok_or_else(|| {
         CoreError::UnsupportedExpect(
             "status_monotonic requires params.goal_id".to_string()
         )
     })?;

     // Filter status events for this goal, ordered by sequence
     let mut status_events: Vec<_> = events
         .iter()
         .filter(|ev| {
             ev.get("type") == Some(&json!("status")) &&
             ev.get("goal_id") == Some(&json!(goal_id))
         })
         .collect();

     // Sort by sequence number to establish ordering
     status_events.sort_by_key(|ev| {
         ev.get("sequence")
             .and_then(|v| v.as_u64())
             .unwrap_or(0)
     });

     // Extract status sequence
     let statuses: Vec<&str> = status_events
         .iter()
         .filter_map(|ev| ev.get("status").and_then(|v| v.as_str()))
         .collect();

     if statuses.is_empty() {
         pass = false;
         violations
             .entry("status_monotonic")
             .or_insert_with(|| json!([]))
             .as_array_mut()
             .unwrap()
             .push(json!({
                 "goal_id": goal_id,
                 "reason": "no status events found"
             }));
         continue;
     }

     // Valid transitions per SPEC_AC04 (action_core.md)
     fn is_valid_transition(from: &str, to: &str) -> bool {
         match (from, to) {
             // From ACCEPTED
             ("STATUS_ACCEPTED", "STATUS_EXECUTING") => true,
             ("STATUS_ACCEPTED", "STATUS_CANCELING") => true,
             ("STATUS_ACCEPTED", "STATUS_SUCCEEDED") => true,
             ("STATUS_ACCEPTED", "STATUS_ABORTED") => true,
             ("STATUS_ACCEPTED", "STATUS_CANCELED") => true,

             // From EXECUTING
             ("STATUS_EXECUTING", "STATUS_CANCELING") => true,
             ("STATUS_EXECUTING", "STATUS_SUCCEEDED") => true,
             ("STATUS_EXECUTING", "STATUS_ABORTED") => true,

             // From CANCELING
             ("STATUS_CANCELING", "STATUS_CANCELED") => true,
             ("STATUS_CANCELING", "STATUS_SUCCEEDED") => true,
             ("STATUS_CANCELING", "STATUS_ABORTED") => true,

             // Terminal states are immutable (no further transitions)
             ("STATUS_SUCCEEDED", _) => false,
             ("STATUS_ABORTED", _) => false,
             ("STATUS_CANCELED", _) => false,

             // Same state (re-publication is allowed)
             (a, b) if a == b => true,

             // All other transitions are invalid
             _ => false
         }
     }

     // Validate monotonicity: each transition must be valid
     let mut regressions = Vec::new();
     for i in 0..statuses.len().saturating_sub(1) {
         let from = statuses[i];
         let to = statuses[i + 1];

         if !is_valid_transition(from, to) {
             regressions.push(json!({
                 "sequence_index": i,
                 "from": from,
                 "to": to,
                 "reason": "invalid transition (regression or illegal state jump)"
             }));
         }
     }

     if !regressions.is_empty() {
         pass = false;
         violations
             .entry("status_monotonic")
             .or_insert_with(|| json!([]))
             .as_array_mut()
             .unwrap()
             .push(json!({
                 "goal_id": goal_id,
                 "status_sequence": statuses,
                 "regressions": regressions
             }));
     }
 }

 Rationale:
 - Filters status events by goal_id and type
 - Sorts by sequence field (backend-global monotonic counter)
 - Validates each transition follows the valid state graph per SPEC_AC04
 - Detects regressions (e.g., EXECUTING → ACCEPTED)
 - Allows re-publication of same state (e.g., ACCEPTED → ACCEPTED)
 - Terminal states cannot transition (validates immutability)

 ---
 5. /harness/scenarios/scenarios_A.json (lines 184-196)

 Replace A06 stub with complete scenario:

 "A06_status_updates_monotonic": {
     "spec_id": "A06",
     "title": "Status updates monotonic",
     "layer": "Core",
     "requires": [
         "actions.basic"
     ],
     "notes": "Validates SPEC_AC05: status updates are monotonically ordered per goal. Canonical path: ACCEPTED → EXECUTING → SUCCEEDED. Sequence numbers validate ordering.",
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
                 "outcome_sequence": "succeed"
             }
         },
         {
             "op": "send_goal",
             "goal_id": "g_mono"
         },
         {
             "op": "wait",
             "ms": 300
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
                         "goal_id": "g_mono",
                         "accepted": true
                     },
                     {
                         "type": "result",
                         "goal_id": "g_mono",
                         "status": "SUCCEEDED"
                     }
                 ]
             }
         },
         {
             "check": "status_monotonic",
             "params": {
                 "goal_id": "g_mono"
             }
         }
     ]
 }

 Validation Strategy:
 - custom checker validates goal acceptance and terminal result appear
 - NEW status_monotonic checker validates complete monotonicity:
   - Filters status events by goal_id
   - Sorts by sequence field (backend-global monotonic counter)
   - Validates each transition follows valid state graph per SPEC_AC04
   - Detects regressions (e.g., EXECUTING → ACCEPTED)
   - Validates terminal immutability (no transitions after terminal state)
 - 300ms wait is sufficient for status propagation (A02 trace shows ~200ms for result)

 ---
 Verification

 Build

 colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release

 Single Run Test

 export ORACLE_BACKEND=prod
 ./harness/scripts/run_harness.sh \
   harness/scenarios/scenarios_A.json \
   /tmp/a06_trace.jsonl \
   /tmp/a06_report.json

 Inspect Status Events

 # Extract status events with sequence, goal_id, status
 grep '"type":"status"' /tmp/a06_trace.jsonl | jq -c '{seq: .sequence, goal: .goal_id, status: .status}'

 Expected output:
 {"seq":6,"goal":"g_mono","status":"STATUS_ACCEPTED"}
 {"seq":8,"goal":"g_mono","status":"STATUS_EXECUTING"}
 {"seq":11,"goal":"g_mono","status":"STATUS_SUCCEEDED"}

 Validation criteria:
 - Sequence numbers strictly increasing (6 < 8 < 11) ✓
 - Status transitions follow valid graph (ACCEPTED → EXECUTING → SUCCEEDED) ✓
 - All three intermediate states observable ✓
 - status_monotonic checker validates no regressions ✓

 Verify Pass/Fail

 jq '.scenarios.A06_status_updates_monotonic.outcome' /tmp/a06_report.json
 # Expected: "PASS"

 Docker Determinism (5-run protocol)

 docker compose build
 for i in {1..5}; do
   docker compose up --abort-on-container-exit
   cp evidence/trace.jsonl /tmp/run_$i.jsonl
   cp evidence/report.json /tmp/report_$i.json
 done

 # Verify identical outcomes
 sha256sum /tmp/report_*.json | awk '{print $1}' | sort -u | wc -l
 # Expected: 1 (all 5 runs produce identical report hash)

 # Verify status event states are consistent
 for i in {1..5}; do
   grep '"type":"status"' /tmp/run_$i.jsonl | jq -r '.state'
 done | sort | uniq -c
 # Expected: Each state appears exactly 5 times (one per run)

 ---
 Risk Mitigation

 CRITICAL SAFETY FIX: Dangling TraceWriter Reference
 - Issue: Original plan captured TraceWriter& w in status subscription callback
 - Problem: w is scoped to op execution; callback fires later → dangling reference → undefined behavior
 - Solution: Buffer events in pending_status_events, drain at safe points where w is valid
 - Result: No dangling references; all events emitted with proper sequence ordering

 Medium Risk: Subscription Timing
 - Issue: If subscription created after goal acceptance, ACCEPTED status might be missed
 - Mitigation: Create subscription before sending goal, drain buffer multiple times

 Medium Risk: Multiple Scenarios in Bundle
 - Issue: uuid_scenario_map might leak between scenarios if not cleared
 - Mitigation: Maps are keyed by UUID, which is unique per goal. Cleanup happens naturally.

 Medium Risk: Status Message Coalescing
 - Issue: Rapid status updates might coalesce on DDS transport
 - Mitigation: 300ms wait gives enough time. Drain buffer at multiple points (send_goal, wait).

 Low Risk: Terminal Status Duplication
 - Issue: Status topic emits SUCCEEDED, and result_callback also emits result
 - Mitigation: Both are valid events. Different semantic meaning (status observation vs result).

 ---
 Success Criteria

 Must achieve all of:
 1. ✅ status_monotonic checker implemented in eval.rs with valid transition graph
 2. ✅ Backend emits status events with correct field name ("status" not "state")
 3. ✅ Scenario A06 passes locally (1/1 run) - no regressions detected
 4. ✅ Scenario A06 passes in Docker (5/5 deterministic runs)
 5. ✅ Status events observable in trace: ACCEPTED, EXECUTING, SUCCEEDED
 6. ✅ Sequence numbers strictly increasing across all events
 7. ✅ No regressions in H00, A01, A02 (baseline scenarios)
 8. ✅ Evidence artifacts non-empty and schema-valid
 9. ✅ Matrix updated only after validation complete

 ---
 Files Modified (Summary)

 1. backend.hpp - Add pending_status_events buffer, status_sub, uuid_scenario_map to BackendState
 2. actions.cpp - Add drain_status_buffer() helper, status subscription (buffers events), drain at safe points
 3. generic.cpp - Modify exec_wait to drain status buffer after sleep
 4. eval.rs - Add status_monotonic checker to validate state transitions and detect regressions
 5. scenarios_A.json - Populate A06 with ops + expects using new checker

 No changes needed:
 - trace.cpp (sequence auto-increment already exists)
 - backend.cpp (dispatch already handles send_goal)
 - trace_event.schema.json (status event type already approved)

 Critical Safety Fix:
 - Status subscription does NOT capture TraceWriter& w (dangling reference risk)
 - Instead, events are buffered and drained during ops when w is valid

 ---
 Next Steps After A06

 Per PLAN.md Track 1 roadmap:
 - A04 (Valid Transitions) - extends status observation to validate full state machine
 - A08 (Cancellation Intent) - adds cancel_request op and observes CANCELING state
 - A07 (Ordering Time-Independent) - validates sequence-based ordering vs wall-clock

 A06 unlocks the status observation infrastructure that these scenarios depend on.