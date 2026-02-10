Plan: P06_unknown_params_rejected — Implementation

 ---
 1. What P06 Proves

 Spec claim: SPEC_PC06 §Undeclared Access Logic — "If allow_undeclared_parameters is false: Set attempts for unknown parameters MUST fail deterministically."

 What the scenario tests: The backend calls set_parameter on the per-scenario node with a parameter name that was never declared. The node is constructed with
 allow_undeclared_parameters(false) pinned explicitly in NodeOptions. The call fails; the outcome is recorded in op_end.detail. The oracle checks that successful: false is present.

 Why this is not a self-confirming fixture (boundaries §2):
 Same reasoning as P03. The rejection is rclcpp's built-in enforcement of allow_undeclared_parameters = false. The backend provides the stimulus (set on an undeclared name); rclcpp
 provides the outcome. No steering.

 Why no actor is needed (boundaries §5):
 set_parameter is a local API on the owning node. The SUT is st.node itself. Same pattern as P03.

 Relationship to P03: P03 tests the declare-collision case (redeclare fails). P06 tests the set-without-declare case (set fails). Together they cover the two primary "undeclared name"
 rejection paths in rclcpp's parameter store.

 Reusability: The set_param op is reused by P04 (batch-set atomicity: set of declared + undeclared params) and any future scenario exercising parameter mutation. The make_parameter
 type-dispatch helper it introduces is the shared entry point for all typed parameter construction.

 ---
 2. Minimal Technical Plan

 2a. New op: set_param

 Where: harness/backends/backend_prod/src/ops/params.cpp — add alongside the existing exec_declare_param. Same file, same namespace. No new source file.

 Helper (file-scope static, above exec_set_param):
 // payload contract: "name" required (checked by caller), "value" required (checked by caller).
 // "type" defaults to "string" if absent — explicit in all current scenarios.
 static rclcpp::Parameter make_parameter(const std::string& name, const json& payload) {
     std::string type = payload.contains("type") ? payload["type"].get<std::string>() : "string";
     if (type == "integer")  return rclcpp::Parameter(name, payload["value"].get<int64_t>());
     if (type == "double")   return rclcpp::Parameter(name, payload["value"].get<double>());
     if (type == "string")   return rclcpp::Parameter(name, payload["value"].get<std::string>());
     if (type == "bool")     return rclcpp::Parameter(name, payload["value"].get<bool>());
     throw BundleError("set_param: unsupported type '" + type + "'");
 }

 Covers the four scalar types. type defaults to "string" when absent — all current scenarios supply it explicitly (P06 uses "integer"). value is mandatory; caller (exec_set_param)
 BundleErrors before reaching this helper if absent. Array types are deferred — no current scenario uses them.

 Main function:
 json exec_set_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
     if (!op.payload || !op.payload->count("name"))
         throw BundleError("set_param: payload missing 'name'");
     if (!op.payload->count("value"))
         throw BundleError("set_param: payload missing 'value'");
     std::string name = (*op.payload)["name"];

     json result;
     result["op"]   = "set_param";
     result["name"] = name;

     try {
         rclcpp::SetParametersResult res = st.node->set_parameter(make_parameter(name, *op.payload));
         result["successful"] = res.successful;
         if (!res.successful && !res.reason.empty())
             result["reason"] = res.reason;
     } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
         result["successful"] = false;
         result["reason"]     = "NotDeclared";
     }
     return result;
 }

 Defensive catch: For a single set_parameter on an undeclared name, rclcpp will typically throw ParameterNotDeclaredException. The SetParametersResult return path is kept as a defensive
 fallback (it is the normal path for set_parameters / set_parameters_atomically; keeping both paths means the op is safe if rclcpp's internal routing changes). The trace output is
 structurally identical either way.

 2b. Wire into backend dispatch + pin NodeOptions

 File: harness/backends/backend_prod/src/backend.cpp

 Four insertions:
 1. Forward declaration in namespace ops { ... } (after line 18, the declare_param forward decl): json exec_set_param(TraceWriter& w, BackendState& st, const std::string& scenario_id,
 const Op& op);
 2. Dispatch branch (after line 51, the declare_param branch): else if (op.op == "set_param") { op_result = ops::exec_set_param(w, st, scenario_id, op); }
 3. Capability (line 87, the caps array): add "ros.params.set" — consistent with P03's "ros.params.declare" (dot-separated hierarchy).
 4. Node creation (line 102): Make allow_undeclared_parameters(false) explicit. Current:
 st.node = std::make_shared<rclcpp::Node>("backend_prod_client_" + id);
 4. Replace with:
 auto opts = rclcpp::NodeOptions()
     .allow_undeclared_parameters(false)
     .automatically_declare_parameters_from_overrides(false);
 st.node = std::make_shared<rclcpp::Node>("backend_prod_client_" + id, opts);
 4. This pins the test condition rather than relying on the rclcpp default. Both options are already false by default in Jazzy, so this is a no-op semantically — it just makes the intent
  explicit and future-proof.

 2c. Scenario rewrite: P06 expects block only

 File: harness/scenarios/scenarios_P.json (P06 entry, lines 133-168)

 The ops block stays identical — one set_param with payload: {name, type, value}. Only expects and notes change.

 Current expects use param_set_request / param_set_response — not in the trace-event type enum. Replace with a single op_end subset-match:

 "expects": [
     {
         "check": "custom",
         "params": {
             "must_observe": [
                 {
                     "type": "op_end",
                     "detail": {
                         "op": "set_param",
                         "name": "param_does_not_exist",
                         "successful": false
                     }
                 }
             ]
         }
     }
 ]

 Why no reason match: The exact rejection text is rclcpp-internal (varies between "return failed result" and "exception message" paths). The spec assertion is successful: false; reason
 is diagnostic only.

 Update notes to: "Setting an undeclared parameter must be rejected by the node. Outcome recorded in op_end.detail; node is constructed with allow_undeclared_parameters(false) pinned in
 NodeOptions."

 ---
 3. Exact File Touch List
 ┌──────────────────────────────────────────────────┬────────────────────────────────────────────────────────────────────────────────┐
 │                       File                       │                                     Change                                     │
 ├──────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/ops/params.cpp │ Add make_parameter helper + exec_set_param                                     │
 ├──────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────┤
 │ harness/backends/backend_prod/src/backend.cpp    │ Forward-declare exec_set_param + dispatch branch + "ros.params.set" capability │
 ├──────────────────────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────┤
 │ harness/scenarios/scenarios_P.json               │ Rewrite P06 expects block; update notes                                        │
 └──────────────────────────────────────────────────┴────────────────────────────────────────────────────────────────────────────────┘
 Files that do NOT change (and why):
 - scenario.schema.json — set_param is already in the op enum (added in a prior phase).
 - CMakeLists.txt — params.cpp is already compiled (added for P03).
 - run_reliability.sh — glob already includes P-series (widened for P03).
 - semantics_P.md — P06 entry text ("SetParameters returns failure") maps directly to successful: false in op_end.detail.
 - parameter_core.md — spec is unchanged.
 - backend.hpp / types.hpp / bundle_parser.cpp — set_param reads from op.payload which is already parsed generically.

 ---
 4. Verification Checklist

 - python3 harness/scripts/validate_bundles.py passes
 - colcon build --packages-select backend_prod compiles cleanly (ParameterNotDeclaredException is in <rclcpp/rclcpp.hpp>, already included)
 - ./tools/testing/run_reliability.sh 1 --scenario P06 produces PASS. Trace contains one op_end with successful: false
 - grep -n "param_set_request\|param_set_response" harness/scenarios/scenarios_P.json returns only P04 hits (P06 is clean)
 - H00, A01, A02, P03 unaffected

 ---
 5. Risks
 #: R1
 Risk: set_parameter throws vs returns on undeclared
 Mitigation: Two-path implementation: try/catch covers both. Trace output is identical regardless of which path fires.
 ────────────────────────────────────────
 #: R2
 Risk: st.node has built-in rclcpp parameters (e.g. use_sim_time); name collision?
 Mitigation: Payload name is param_does_not_exist — no collision with any standard rclcpp parameter.
 ────────────────────────────────────────
 #: R3
 Risk: P03's declare_param also runs on st.node; does the explicit NodeOptions affect it?
 Mitigation: No. allow_undeclared_parameters(false) only gates set_parameter on undeclared names. declare_parameter is unaffected. automatically_declare_parameters_from_overrides(false)
   prevents --ros-args -p overrides from auto-declaring, which is the right default for a test harness node.
 ────────────────────────────────────────
 #: R4
 Risk: Oracle nested-detail matching (same as P03 R3)
 Mitigation: Pre-existing limitation if present; not introduced here.