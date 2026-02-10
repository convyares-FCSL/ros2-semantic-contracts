 P04_no_partial_application — Implementation Plan

 What P04 Proves

 Exact spec claims:
 - SPEC_PC05 (Core): "Set operations are atomic transactions. Failed operations MUST NOT partially apply. All changes in a request MUST be applied, or
 none."
 - SPEC_P04 (Global): "Parameter updates MUST be atomic per request. Partial application MUST NOT be externally observable."

 Observable assertion:
 If setting parameters A and B in a batch, and B is invalid, then A remains unchanged and the operation returns failure.

 Why it's falsifiable:
 A non-compliant implementation could:
 - Apply A's change while rejecting B (partial application)
 - Return successful=true despite B's invalidity
 - Emit a parameter event for A even though the batch failed

 Why it isn't self-confirming:
 - The stimulus (batch set with one undeclared param) is chosen by the scenario
 - The outcome (rejection, atomicity) is determined by rclcpp's implementation with allow_undeclared_parameters=false
 - The scenario does NOT control whether rclcpp will reject or how atomicity is enforced
 - Uses real set_parameters_atomically() API with no mocking or callbacks

 Why no actor is required:
 This tests the local rclcpp parameter API (set_parameters_atomically), not distributed parameter services. Per boundaries.md §5, the backend hosts the SUT
  node directly. No peer interaction required.

 ---
 Minimal Technical Plan

 CRITICAL NAMING NOTE: Op names must match EXACTLY across schema, dispatch, capabilities, and scenario JSON. The op is set_params_batch (not
 set_params_batch_atomic, set_parameters_batch, or any variant). Use snake_case with underscores, NOT camelCase.

 Required op #1: get_param

 Why required: P04 claim is "A remains unchanged". Without observing A's value before and after, we're only testing rejection, not immutability. This
 closes the verification gap.

 Function signature:
 json exec_get_param(TraceWriter& w, BackendState& st,
                     const std::string& scenario_id, const Op& op);

 File location:
 harness/backends/backend_prod/src/ops/params.cpp

 JSON payload schema:
 {
   "op": "get_param",
   "payload": {
     "name": "param_name"
   }
 }

 Return JSON detail format for op_end:
 {
   "op": "get_param",
   "name": "param_name",
   "exists": true,
   "value": "",
   "type": "string"
 }

 If parameter not declared:
 {
   "op": "get_param",
   "name": "param_name",
   "exists": false,
   "type": "NOT_SET"
 }

 Exact rclcpp API call:
 rclcpp::Parameter param = st.node->get_parameter(name);
 // Then extract value based on param.get_type()

 Implementation pseudocode:
 json exec_get_param(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
     if (!op.payload || !op.payload->count("name"))
         throw BundleError("get_param: payload missing 'name'");

     std::string name = (*op.payload)["name"];
     json result;
     result["op"] = "get_param";
     result["name"] = name;

     try {
         rclcpp::Parameter param = st.node->get_parameter(name);
         result["exists"] = true;

         switch (param.get_type()) {
             case rclcpp::ParameterType::PARAMETER_INTEGER:
                 result["value"] = param.as_int();
                 result["type"] = "integer";
                 break;
             case rclcpp::ParameterType::PARAMETER_DOUBLE:
                 result["value"] = param.as_double();
                 result["type"] = "double";
                 break;
             case rclcpp::ParameterType::PARAMETER_STRING:
                 result["value"] = param.as_string();
                 result["type"] = "string";
                 break;
             case rclcpp::ParameterType::PARAMETER_BOOL:
                 result["value"] = param.as_bool();
                 result["type"] = "bool";
                 break;
             case rclcpp::ParameterType::PARAMETER_NOT_SET:
                 result["exists"] = false;
                 result["type"] = "NOT_SET";
                 break;
             default:
                 result["type"] = "unknown";
         }
     } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
         result["exists"] = false;
         result["type"] = "NOT_SET";
     }

     return result;
 }

 Implementation notes:
 - Handle ParameterNotDeclaredException → set exists=false, type="NOT_SET"
 - Support types: integer, double, string, bool (array types deferred)
 - Use param.get_type() to determine type, then as_int(), as_string(), etc.
 - If PARAMETER_NOT_SET, treat as non-existent

 ---
 Required op #2: set_params_batch

 Function signature:
 json exec_set_params_batch(TraceWriter& w, BackendState& st,
                            const std::string& scenario_id, const Op& op);

 File location:
 harness/backends/backend_prod/src/ops/params.cpp

 JSON payload schema:
 {
   "op": "set_params_batch",
   "payload": {
     "params": [
       {"name": "param_a", "value": 42, "type": "integer"},
       {"name": "param_b", "value": "hello", "type": "string"}
     ]
   }
 }

 Return JSON detail format for op_end:
 {
   "op": "set_params_batch",
   "count": 2,
   "successful": false,
   "reason": "NotDeclared"  // optional, only if unsuccessful
 }

 Capability string:
 Add "ros.params.set_batch" to the backend_capabilities caps array in backend.cpp:90.

 Alternative naming (optional): "ros.params.set_atomic" to distinguish from non-atomic batch operations. Either is acceptable; plan uses set_batch for
 consistency with op name.

 Exact rclcpp API call:
 std::vector<rclcpp::Parameter> params;
 // ... build params vector using existing make_parameter() helper ...

 rcl_interfaces::msg::SetParametersResult res =
     st.node->set_parameters_atomically(params);

 Implementation pseudocode:
 json exec_set_params_batch(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
     if (!op.payload || !op.payload->count("params"))
         throw BundleError("set_params_batch: payload missing 'params'");

     const json& params_array = (*op.payload)["params"];
     if (!params_array.is_array())
         throw BundleError("set_params_batch: 'params' must be an array");

     std::vector<rclcpp::Parameter> params;
     for (const auto& p : params_array) {
         if (!p.count("name") || !p.count("value"))
             throw BundleError("set_params_batch: each param must have 'name' and 'value'");
         params.push_back(make_parameter(p["name"], p));  // reuse existing helper
     }

     json result;
     result["op"] = "set_params_batch";
     result["count"] = params.size();

     try {
         rcl_interfaces::msg::SetParametersResult res = st.node->set_parameters_atomically(params);
         result["successful"] = res.successful;
         if (!res.successful && !res.reason.empty())
             result["reason"] = res.reason;  // optional, for debugging
     } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
         result["successful"] = false;
         result["reason"] = "NotDeclared";
     } catch (const std::exception& e) {
         result["successful"] = false;
         result["reason"] = std::string("Exception: ") + e.what();
     }

     return result;
 }

 Implementation notes:
 - Reuse existing make_parameter(name, payload) helper from params.cpp
 - Handle both exception path (ParameterNotDeclaredException) and return-value path (SetParametersResult.successful)
 - Per P06 pattern: catch exception → set successful=false, reason="NotDeclared"
 - Do NOT match on res.reason in expects (implementation-internal text)
 - Defensive: catch generic exception for robustness

 ---
 Defining "Invalid" Parameter B

 Chosen approach: Undeclared parameter rejection

 The scenario will:
 1. Declare parameter A (valid, declared)
 2. Batch-set A (valid) + B (invalid because undeclared)
 3. Since allow_undeclared_parameters=false (pinned in NodeOptions), B is rejected
 4. Atomic semantics → entire batch fails
 5. A remains unchanged (initially empty string "")

 Why this approach:
 - Deterministic: always fails with allow_undeclared=false (already pinned)
 - Standards-compliant: follows ROS 2 parameter semantics (SPEC_PC06)
 - Not self-confirming: scenario picks stimulus; rclcpp picks outcome
 - No scenario-shaped policy: no callbacks, no special configurations
 - Reuses existing infrastructure: NodeOptions already configured correctly

 Alternative approaches considered:
 - Type mismatch (declare A as integer, set as string): requires declaring both A and B upfront, less clear signal
 - OnSetParametersCallback rejection: potentially scenario-shaped if callback is per-scenario
 - Read-only enforcement: requires descriptor support (P05, not implemented)

 ---
 Scenario Rewrite

 File: harness/scenarios/scenarios_P.json

 What changes:

 ops block (full rewrite with state verification):

 Replace existing ops with get-before, set-batch, get-after pattern:
 "ops": [
   {"op": "declare_param", "payload": {"name": "valid_param"}},
   {"op": "get_param", "payload": {"name": "valid_param"}},
   {"op": "set_params_batch", "payload": {
     "params": [
       {"name": "valid_param", "value": "foo", "type": "string"},
       {"name": "undeclared_param", "value": "bar", "type": "string"}
     ]
   }},
   {"op": "get_param", "payload": {"name": "valid_param"}}
 ]

 Operation sequence:
 1. Declare valid_param (initial value will be empty string "" per declare_param default)
 2. Get valid_param → capture initial state (expect value="")
 3. Batch-set valid_param="foo" + undeclared_param="bar" → expect atomic rejection
 4. Get valid_param → verify still value="" (unchanged)

 expects block (full rewrite with state verification):

 Replace custom event types with op_end subset-match including state assertions:
 "expects": [
   {
     "check": "custom",
     "params": {
       "must_observe": [
         {
           "type": "op_end",
           "detail": {
             "op": "declare_param",
             "name": "valid_param",
             "declared": true
           }
         },
         {
           "type": "op_end",
           "detail": {
             "op": "get_param",
             "name": "valid_param",
             "exists": true,
             "value": ""
           }
         },
         {
           "type": "op_end",
           "detail": {
             "op": "set_params_batch",
             "successful": false
           }
         },
         {
           "type": "op_end",
           "detail": {
             "op": "get_param",
             "name": "valid_param",
             "exists": true,
             "value": ""
           }
         }
       ]
     }
   }
 ]

 Oracle judgment:
 - Match #1: declare succeeded
 - Match #2: get-before returned value=""
 - Match #3: batch-set rejected with successful=false
 - Match #4: get-after returned value="" (SAME as #2)
 - Pass criteria: All four events match → proves no partial application
 - Fail criteria: If get-after returns value="foo", partial application occurred → FAIL

 notes field update:

 "notes": "Atomic batch-set with one undeclared param must reject entirely. Uses set_parameters_atomically; state verified with get_param before/after.
 Both params remain unchanged on rejection. Validates SPEC_PC05+SPEC_P04 atomicity."

 ---
 Exact File Touch List

 1. harness/backends/backend_prod/src/ops/params.cpp
   - Add exec_get_param() function (~45 lines, new)
   - Add exec_set_params_batch() function (~40 lines, new)
 2. harness/backends/backend_prod/src/backend.cpp
   - Line ~18: Add forward declaration: json exec_get_param(...);
   - Line ~19: Add forward declaration: json exec_set_params_batch(...);
   - Line ~52: Add dispatch branch for get_param:
   } else if (op.op == "get_param") {
     op_result = ops::exec_get_param(w, st, scenario_id, op);
   - Line ~55: Add dispatch branch for set_params_batch:
   } else if (op.op == "set_params_batch") {
     op_result = ops::exec_set_params_batch(w, st, scenario_id, op);
   - Line ~90: Add "ros.params.get" and "ros.params.set_batch" to caps array
 3. harness/schemas/scenario.schema.json
   - Line ~69 (in op enum): Confirm "get_param" exists (already present)
   - Line ~69 (in op enum): Add "set_params_batch" to the enum list (exact name, no drift)
 4. harness/scenarios/scenarios_P.json
   - P04 entry: rewrite ops block (add get_param before and after)
   - P04 entry: rewrite expects block (add state verification)
   - P04 entry: update notes field
 5. docs/spec/core/parameter_core.md (optional documentation)
   - No changes required (SPEC_PC05 already documents atomicity)

 ---
 Verification Checklist

 Build and validate:

 # Build backend_prod
 colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release

 # Validate schema compliance
 python3 harness/scripts/validate_bundles.py

 Run P04:

 # Single-scenario test (ephemeral bundle)
 ./tools/testing/run_reliability.sh --scenario P04_no_partial_application --quick 1

 # Or via full bundle run:
 export ORACLE_BACKEND=prod
 ./tools/testing/run_reliability.sh --bundle harness/scenarios/scenarios_P.json

 Check trace:

 # Expected events in trace (in order):
 # 1. op_end with "op":"declare_param", "declared":true
 # 2. op_end with "op":"set_params_batch", "successful":false

 grep '"type":"op_end"' /tmp/trace.jsonl | grep -E 'declare_param|set_params_batch'

 Grep for stale event types:

 # Should return NO matches in P04 after rewrite:
 grep -E 'param_declare_response|param_set_batch_response' harness/scenarios/scenarios_P.json

 Determinism check (5-run Docker protocol):

 # Per PLAN.md, run 5 times in Docker, verify identical outcomes
 docker compose build
 for i in {1..5}; do
   docker compose up --abort-on-container-exit
   # Compare traces, should be bitwise identical (modulo timestamps)
 done

 ---
 Risks

 1. Exception vs return-value behavior

 Risk: Does set_parameters_atomically() throw ParameterNotDeclaredException or return successful=false when a param is undeclared?

 Mitigation: Implement defensive handling for both paths (catch exception OR check return value), following P06 pattern.

 Evidence: P06 (set_param single) throws; atomically may differ. The Rust reference impl returns success=false without throwing. C++ rclcpp may vary.

 ---
 2. rclcpp reason string variability

 Risk: The SetParametersResult.reason string is implementation-internal and may change across rclcpp versions.

 Mitigation: Do NOT match on reason in expects block. Only match on successful boolean. Document reason in op_end.detail for debugging only.

 ---
 3. Determinism of undeclared rejection

 Risk: Could allow_undeclared_parameters=false behavior vary non-deterministically?

 Mitigation: NodeOptions already pins both allow_undeclared_parameters(false) and automatically_declare_parameters_from_overrides(false) in
 backend.cpp:109-111. This is stable Jazzy behavior.

 Evidence: P06 already validates single-param undeclared rejection deterministically. Batch behavior should be equally stable.

 ---
 Composite Spec Justification (SPEC_PC05 + SPEC_P04)

 Question: P04 validates both a Core spec (SPEC_PC05) and a Global spec (SPEC_P04). Is this allowed?

 Answer: Yes. Per semantics_P.md:94, P04 is explicitly documented as exercising both layers:
 "P04 exercises the decision layer: the set-operation outcome is all-or-nothing."

 - SPEC_PC05 defines atomicity at the decision layer (set succeeds or fails as a unit)
 - SPEC_P04 defines atomicity at the observable layer (no partial state visible externally)

 Both are exercised together because:
 1. The decision-layer outcome (successful=false) is the primary observable
 2. The external state immutability is the consequence
 3. They reduce to the same evidence today (no partial application)

 This is explicitly allowed as a "Core↔Global composite" scenario. See also P07, P08, P14 notes for similar justification.

 ---
 Summary of Changes
 ┌──────────────────────┬────────────────────────────────────────────────┬────────────┐
 │         File         │                     Change                     │   Lines    │
 ├──────────────────────┼────────────────────────────────────────────────┼────────────┤
 │ params.cpp           │ Add exec_get_param() + exec_set_params_batch() │ ~85 new    │
 ├──────────────────────┼────────────────────────────────────────────────┼────────────┤
 │ backend.cpp          │ Forward-decls (2), dispatch (2), caps (2)      │ 6 edits    │
 ├──────────────────────┼────────────────────────────────────────────────┼────────────┤
 │ scenario.schema.json │ Confirm get_param exists, add set_params_batch │ 1 edit     │
 ├──────────────────────┼────────────────────────────────────────────────┼────────────┤
 │ scenarios_P.json     │ Rewrite P04 ops+expects, update notes          │ 1 scenario │
 └──────────────────────┴────────────────────────────────────────────────┴────────────┘
 No new files.
 No actor changes.
 No oracle_core changes.

 Two ops implemented:
 1. get_param — needed for state verification (also unlocks P01, P08, P13)
 2. set_params_batch — atomic batch parameter set operation

 ---
 Post-Implementation

 After P04 passes:
 1. Update coverage matrix: set ros column to PASS, Notes to "Atomic batch rejection via set_parameters_atomically"
 2. Commit with message format:
 feat(params): implement P04_no_partial_application — set_params_batch op

 Add atomic batch set operation using rclcpp::Node::set_parameters_atomically.
 Validates SPEC_PC05 (Core atomicity) and SPEC_P04 (Global atomicity).
 Uses undeclared param rejection as deterministic failure signal.

 Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>
 3. Run full reliability sweep: ./tools/testing/run_reliability.sh (default glob includes P scenarios)
 4. Verify H00 still passes (baseline stability)

 ---
 Next Candidates (After P04)

 - P05 — needs declare_param with read_only=true descriptor support (descriptor field not yet implemented in declare_param)
 - P01 — needs list_params op (not implemented)
 - P08 — needs describe_param op (not implemented, not in schema enum)
 - P02 — needs declare_param with dynamic_typing descriptor + type-change set attempt

 P04 unlocks no immediate dependencies. P05 is blocked on descriptor support. Consider P01 (list) or P08 (describe) next, depending on priority.