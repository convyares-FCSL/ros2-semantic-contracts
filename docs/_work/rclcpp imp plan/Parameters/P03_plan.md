Plan: P03_redeclaration_fails — Implementation

 ---
 1. What P03 Proves

 Spec claim: SPEC_PC04 §Declaration Logic — "Declaring an already-declared parameter MUST fail deterministically (outcome: AlreadyDeclared)."

 What the scenario tests: The backend's per-scenario node calls rclcpp::Node::declare_parameter() twice with the same name. The first call succeeds; the second throws
 rclcpp::ParameterAlreadyDeclaredException. Both outcomes are recorded in op_end.detail. The oracle checks that both events are present and correct.

 Why this is not a self-confirming fixture (boundaries §2):
 The rejection is rclcpp's built-in invariant — the backend does not steer it. The scenario provides the stimulus (same name twice); rclcpp provides the outcome. The backend only
 observes and records. Decision autonomy belongs to rclcpp, not the scenario.

 Why no actor is needed (boundaries §5):
 declare_parameter is a local API on the owning node. There is no remote peer; the SUT is the node. Per §5: "If the spec concerns server semantics, the backend may host the server as the
  SUT."

 Reusability: The declare_param op is a prerequisite setup step for P04 (atomicity: pre-declare before batch-set), P05 (read-only: pre-declare with read_only descriptor), and P06
 (undeclared-access: contrast declared vs undeclared). It is not scenario-specific.

 ---
 2. Minimal Technical Plan

 2a. New op: declare_param

 Where: src/ops/params.cpp — stub file already exists (1 empty line), NOT currently compiled (absent from CMakeLists). This is the designated home for parameter ops.

 Signature (follows the exec_get_clock / exec_start_actor return-json pattern):
 json exec_declare_param(TraceWriter&, BackendState&, const std::string& scenario_id, const Op& op)

 Logic:
 1. Extract name from op.payload["name"] (BundleError if payload absent or missing name).
 2. Call st.node->declare_parameter<std::string>(name, "").
   - On success: return { "op": "declare_param", "name": name, "declared": true }.
   - On rclcpp::ParameterAlreadyDeclaredException: return { "op": "declare_param", "name": name, "declared": false, "reason": "AlreadyDeclared" }.
 3. The returned json becomes op_end.detail via the existing dispatch pattern in backend.cpp:54-58.

 Why declare_parameter<std::string>(name, ""): P03 does not care about the parameter value or type — only that the name collision is caught. A fixed string default is the simplest valid
 overload. Future scenarios that need typed declarations (P04, P05) can extend the payload with value/type fields; the op implementation can branch on their presence at that point.

 Include: <rclcpp/rclcpp.hpp> is already available via ../backend.hpp. ParameterAlreadyDeclaredException is declared in that header.

 2b. Schema: add declare_param to op enum

 File: harness/schemas/scenario.schema.json
 Change: Insert "declare_param" into the op enum array (currently lines 60-72). Place it after "get_param" — keeping the parameter ops (set_param, get_param, declare_param) grouped
 together.

 Current enum:
 "start_actor", "stop_actor", "send_goal", "wait",
 "emit_feedback", "emit_status", "emit_result", "request_cancel",
 "set_param", "get_param", "call_service"
 After:
 "start_actor", "stop_actor", "send_goal", "wait",
 "emit_feedback", "emit_status", "emit_result", "request_cancel",
 "set_param", "get_param", "declare_param", "call_service"

 2c. Scenario rewrite: P03 expects block only

 File: harness/scenarios/scenarios_P.json (lines 22-71)

 The ops block stays identical — two declare_param ops with payload.name: "test_redeclare_param". Only the expects block and notes change.

 Current expects reference param_declare_request / param_declare_response event types — these are not in the approved trace-event type enum. Replace with op_end subset-matches on the
 fields that exec_declare_param returns:

 "expects": [
     {
         "check": "custom",
         "params": {
             "must_observe": [
                 {
                     "type": "op_end",
                     "detail": {
                         "op": "declare_param",
                         "name": "test_redeclare_param",
                         "declared": true
                     }
                 },
                 {
                     "type": "op_end",
                     "detail": {
                         "op": "declare_param",
                         "name": "test_redeclare_param",
                         "declared": false,
                         "reason": "AlreadyDeclared"
                     }
                 }
             ]
         }
     }
 ]

 Update notes to: "Declaring the same parameter twice must fail deterministically. Outcomes recorded in op_end.detail; second declare triggers rclcpp ParameterAlreadyDeclaredException."

 2d. Wire into backend dispatch

 File: harness/backends/backend_prod/src/backend.cpp

 Two surgical insertions:
 1. Forward declaration in the namespace ops { ... } block (after line 17): json exec_declare_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op);
 2. Dispatch branch before the final else (after line 48, the stop_actor branch): else if (op.op == "declare_param") { op_result = ops::exec_declare_param(w, st, scenario_id, op); }

 2e. Compile the new source

 File: harness/backends/backend_prod/CMakeLists.txt

 Add src/ops/params.cpp to the backend_prod executable sources list (line 13-23). Place it after src/ops/generic.cpp (line 20).

 ---
 3. Exact File Touch List
 ┌───────────────────────────────────────────────────────┬────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │                         File                          │                                                     Change                                                     │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ harness/schemas/scenario.schema.json                  │ Add "declare_param" to op enum (after "get_param")                                                             │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ harness/scenarios/scenarios_P.json                    │ Rewrite P03 expects block; update notes                                                                        │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/ops/params.cpp      │ Implement exec_declare_param (file exists as empty stub)                                                       │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/backend.cpp         │ Forward-declare + dispatch branch for declare_param                                                            │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/CMakeLists.txt          │ Add src/ops/params.cpp to sources                                                                              │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ tools/testing/run_reliability.sh                      │ Widen default bundle glob from [AH] to [AHP] (line 98)                                                         │
 ├───────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
 │ docs/provenance/scenario/scenario_execution_matrix.md │ P03 row: change stub/ros columns from PASS to N/A (stale; the declare_param op did not exist before this plan) │
 └───────────────────────────────────────────────────────┴────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
 Files that do NOT change:
 - backend.hpp / types.hpp — no new struct fields; declare_param reads name from op.payload which is already parsed generically.
 - bundle_parser.cpp — payload is already parsed as a raw json object (line 60-62).
 - action_server.cpp / launch_peer.sh — P03 uses no actor.
 - semantics_P.md — P03 entry text ("declare_parameter call throws or returns failure") maps directly to declared: false in op_end.detail. No rewording needed.
 - parameter_core.md — spec is unchanged.

 ---
 4. Reliability + CI Integration

 Glob widening

 run_reliability.sh line 98: Python glob inside resolve_bundles(). Change:
 bundles = sorted(glob.glob('harness/scenarios/scenarios_[AH]*.json'))
 to:
 bundles = sorted(glob.glob('harness/scenarios/scenarios_[AHP]*.json'))

 Effect: P03 (and any other P scenario with non-empty ops + expects) will be included in default reliability runs.

 Expected noise from other P skeletons: P04, P06, P08, P12 have non-empty ops + expects but use unimplemented ops (set_params_batch, set_param, describe_param). They will hit
 "unsupported op" in the backend dispatch and produce INFRA_FAIL in the runner. This is intentional: it makes coverage gaps visible in the default run rather than hiding them behind
 manual --scenario flags. The runner already counts INFRA_FAIL separately and does not conflate it with semantic failures.

 Verification checklist

 - python3 harness/scripts/validate_bundles.py passes (P03 has all required fields; boundary check is Axx-only)
 - colcon build --packages-select backend_prod compiles cleanly (params.cpp added; rclcpp::ParameterAlreadyDeclaredException is in the rclcpp header already included)
 - ./tools/testing/run_reliability.sh 1 --scenario P03 produces PASS. Trace contains two op_end events: first with declared:true, second with declared:false + reason:AlreadyDeclared
 - grep -n "param_declare_request\|param_declare_response" harness/scenarios/scenarios_P.json returns nothing
 - Coverage matrix P03 row updated
 - H00 smoke test unaffected

 ---
 5. Risks
 #: R1
 Risk: st.node already has an action client; does declare_parameter conflict?
 Mitigation: No. declare_parameter is orthogonal to action clients — they share the node but use independent subsystems.
 ────────────────────────────────────────
 #: R2
 Risk: ParameterAlreadyDeclaredException header availability
 Mitigation: Declared in <rclcpp/rclcpp.hpp>, already included transitively via backend.hpp. No new include needed in params.cpp beyond "../backend.hpp".
 ────────────────────────────────────────
 #: R3
 Risk: Oracle custom check does not match on nested detail fields
 Mitigation: The subset-match pattern on nested fields is used by existing scenarios (A01/A02 result callbacks produce nested detail). If the oracle matcher does not support it, P03 will

   FAIL — but that is a pre-existing oracle limitation, not introduced here.
 ────────────────────────────────────────
 #: R4
 Risk: Someone adds a skeleton scenario to the allowlist before it is ready
 Mitigation: The allowlist file comment says "add here only after 5/5 Docker pass." Enforcement is by convention; the runner does not gate on a separate "ready" flag. Keep the list short and deliberate.