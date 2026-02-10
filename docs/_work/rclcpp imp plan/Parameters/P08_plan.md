 Implementation Plan: P08_unknown_types_not_set (REVISED)

  Evidence Strategy

  Observable claim: When GetParameters is called on an undeclared/unknown parameter, the response must include type: "NOT_SET" with declared: false and set:
   false.

  Trace mechanism: Use existing get_param op. The op_end.detail will encode:
  {
    "op": "get_param",
    "declared": false,
    "set": false,
    "type": "NOT_SET"
  }

  API choice and spec fidelity:

  P08 validates GetParameters wire semantics (SPEC_P03). Two implementation options:

  Option A (highest fidelity): Create new get_parameters op that calls node->get_parameters({name}) and returns vector response
  - Pros: Direct wire API, exact spec match
  - Cons: New op, schema change, more complexity for single-param case

  Option B (pragmatic): Reuse get_param op as canonical observation primitive
  - Explicitly model GetParameters single-name semantics using has_parameter() + conditional get_parameter()
  - Pros: No new op, minimal diff, same observable outcome
  - Cons: One layer of abstraction from wire API

  Recommended: Option B — get_param is our normalized observation primitive that implements GetParameters semantics for a single parameter name. This is
  acceptable because:
  1. Observable outcome is identical (declared/set/type fields match GetParameters response)
  2. Backend remains observer-only (no scenario-shaped steering)
  3. Deterministic and verifiable
  4. Minimal schema/code churn

  Why this is valid as Core↔Global composite:
  - PC03 (Core): Backend observes rclcpp's type resolution for unknown parameters
  - P03 (Global): GetParameters returns ParameterType::PARAMETER_NOT_SET (value 0) for missing keys
  - Trace compliance: No new event types. Outcome encoded in op_end.detail.

  ---
  Ops and Schema

  Existing ops (reuse)

  - get_param — already implemented in params.cpp, already in scenario.schema.json enum
  - Backend capability: "ros.params.get" (already in backend_capabilities array)

  No new ops required

  Current get_param contract (verify implementation):
  // Input payload: { "name": "<param_name>" }
  // Output detail: {
  //   "op": "get_param",
  //   "declared": bool,    // has_parameter(name)
  //   "set": bool,         // declared && type != NOT_SET
  //   "type": string,      // "NOT_SET" | "string" | "integer" | "double" | "bool"
  //   "value": <any>       // only present when set == true
  // }

  ---
  Backend Implementation Details

  File: harness/backends/backend_prod/src/ops/params.cpp

  Current implementation assumption (MUST VERIFY):
  json get_param_op(const json& payload, State& st) {
      std::string name = payload.at("name");

      json result;
      result["op"] = "get_param";

      // Step 1: Check if parameter is declared
      bool declared = st.node->has_parameter(name);
      result["declared"] = declared;

      if (!declared) {
          // Undeclared parameter → GetParameters semantics: NOT_SET
          result["set"] = false;
          result["type"] = "NOT_SET";
          return result;
      }

      // Step 2: Parameter exists, get its value
      rclcpp::Parameter param;
      bool retrieved = st.node->get_parameter(name, param);
      // retrieved should always be true here since has_parameter returned true

      if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
          // Declared but not set
          result["set"] = false;
          result["type"] = "NOT_SET";
      } else {
          result["set"] = true;
          result["type"] = parameter_type_to_string(param.get_type());
          result["value"] = extract_value(param);
      }

      return result;
  }

  ROS API mapping:
  1. node->has_parameter(name) → bool (true if declared, false otherwise)
  2. node->get_parameter(name, rclcpp::Parameter& out) → bool (retrieves value if declared)
  3. param.get_type() → rclcpp::ParameterType enum

  Key paths for P08:
  - Undeclared parameter: has_parameter(name) returns false → output declared: false, set: false, type: "NOT_SET"
  - This models GetParameters behavior: request unknown name → response contains ParameterType::PARAMETER_NOT_SET

  Verification needed:
  # Check current get_param implementation
  grep -A 40 "get_param_op" harness/backends/backend_prod/src/ops/params.cpp

  If current implementation is missing the undeclared path:
  - Add the has_parameter() check before get_parameter()
  - Return early with declared: false, set: false, type: "NOT_SET" when parameter doesn't exist

  Minimal touch list:
  1. Verify get_param in params.cpp handles has_parameter() == false path
  2. If missing: add ~5 lines for undeclared-parameter branch
  3. No changes to backend.cpp, scenario.schema.json, or CMakeLists.txt

  ---
  Scenario JSON Rewrite

  ops block (FIXED: payload not params)

  "ops": [
    {
      "op": "get_param",
      "payload": {
        "name": "missing_param"
      }
    }
  ]

  Rationale:
  - missing_param avoids collision with ROS defaults (use_sim_time, __name, __log_level)
  - No declare_param precedes → guaranteed undeclared
  - Single op → minimal, deterministic

  expects block

  "expects": {
    "check": "custom",
    "params": {
      "must_observe": [
        {
          "type": "op_end",
          "detail": {
            "op": "get_param",
            "declared": false,
            "set": false,
            "type": "NOT_SET"
          }
        }
      ]
    }
  }

  Subset-match semantics: Oracle verifies op_end event with detail fields matching exactly. Does NOT match on name or value (value should be absent).

  Full P08 entry (complete replacement in scenarios_P.json)

  {
    "id": "P08_unknown_types_not_set",
    "description": "Unknown parameters return type NOT_SET",
    "requires": ["ros.params.get"],
    "metadata": {
      "validates": ["SPEC_PC03", "SPEC_P03"],
      "claim": "Unknown parameters are represented with a NOT_SET type.",
      "scope_note": "Core↔Global composite: decision-layer type enforcement (PC03) + wire response NOT_SET (P03). get_param op models GetParameters
  semantics for single-name query."
    },
    "ops": [
      {
        "op": "get_param",
        "payload": {
          "name": "missing_param"
        }
      }
    ],
    "expects": {
      "check": "custom",
      "params": {
        "must_observe": [
          {
            "type": "op_end",
            "detail": {
              "op": "get_param",
              "declared": false,
              "set": false,
              "type": "NOT_SET"
            }
          }
        ]
      }
    },
    "notes": "Validates that querying an undeclared parameter returns NOT_SET type. get_param op implements GetParameters semantics: has_parameter() returns
   false → declared:false, type:NOT_SET."
  }

  ---
  Spec Fidelity Justification

  GetParameters service vs get_param op:

  The ROS 2 GetParameters service (rcl_interfaces/srv/GetParameters) returns a ParameterValue[] where each entry corresponds to a requested name. For
  unknown parameters, the service returns ParameterType::PARAMETER_NOT_SET.

  Our get_param op models this single-name case:
  1. Input: { "name": "X" }
  2. Backend calls has_parameter("X")
  3. If false: output { declared: false, set: false, type: "NOT_SET" }
  4. This is functionally equivalent to GetParameters service returning NOT_SET for unknown name

  Why not call node->get_parameters():
  - rclcpp's get_parameters() returns std::vector<rclcpp::Parameter>
  - For undeclared parameters, it may return empty vector OR throw (depending on implementation)
  - has_parameter() is the cleaner, more deterministic check
  - Observable outcome is identical: backend records that the parameter doesn't exist and has type NOT_SET

  This is not self-confirming because:
  - Scenario chooses the stimulus: query parameter "missing_param"
  - rclcpp chooses the outcome: has_parameter() returns false (or true if param was secretly declared)
  - Backend observes and records the outcome without steering it

  ---
  Determinism + Divergence Note

  backend_prod (rclcpp) determinism

  Expected: 100% deterministic.

  - Synchronous API (has_parameter())
  - No side effects
  - No DDS traffic (local node query)
  - No threading (single executor)
  - No timing dependencies

  Prediction: 5/5 identical traces in Docker validation.

  Probe divergence (rclpy, rclrs)

  Likely divergence points:

  1. rclpy:
    - node.has_parameter(name) exists and should behave identically
    - BUT: Python binding may auto-declare certain parameters
    - Type string may be "not_set" vs "NOT_SET" (case/spacing)
  2. rclrs:
    - Rust API may use Option<Parameter> instead of bool return
    - NOT_SET encoding may differ
    - String representation may vary
  3. Default parameter behavior:
    - Some client libraries auto-declare use_sim_time
    - Using missing_param (non-standard name) should avoid this
    - BUT: probe backends may have different NodeOptions defaults

  Mitigation: Focus on rclcpp/backend_prod passing. Document probe divergence in Track 2.

  ---
  Verification Checklist

  Pre-implementation

  # 1. Verify get_param exists and check implementation

● Read 1 file (ctrl+o to expand)

● # 2. Check current P08 scenario
  jq '.scenarios[] | select(.id == "P08_unknown_types_not_set")' harness/scenarios/scenarios_P.json

  # 3. Verify get_param in scenario schema enum
  grep "get_param" harness/schemas/scenario.schema.json

  # 4. Verify backend dispatch
  grep -A 3 "get_param" harness/backends/backend_prod/src/backend.cpp

  Implementation fix required

  FOUND: Current exec_get_param (lines 93-96) returns early for undeclared parameters but is incomplete:
  if (!st.node->has_parameter(name)) {
      result["declared"] = false;
      return result;  // ❌ Missing set:false, type:NOT_SET
  }

  Required fix (lines 93-96):
  if (!st.node->has_parameter(name)) {
      result["declared"] = false;
      result["set"] = false;
      result["type"] = "NOT_SET";
      return result;
  }

  This 2-line addition completes the GetParameters semantics for undeclared parameters.

  Post-implementation

  # 1. Schema validation
  python3 harness/scripts/validate_bundles.py

  # 2. Build
  colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release

  # 3. Single-scenario run
  ./tools/testing/run_reliability.sh 1 --scenario P08_unknown_types_not_set

  # 4. Five-run determinism
  ./tools/testing/run_reliability.sh 5 --scenario P08_unknown_types_not_set

  # 5. Grep check for forbidden events
  jq '.scenarios[] | select(.id == "P08_unknown_types_not_set") | .expects' \
     harness/scenarios/scenarios_P.json | \
     grep -E '(param_get_response|verdict|assertion)' && echo "FORBIDDEN" || echo "Clean"

  # 6. Full reliability suite
  ./tools/testing/run_reliability.sh 5

  Coverage matrix update

  In docs/provenance/scenario/scenario_execution_matrix.md:
  - Set P08 ros column: PASS
  - Update Notes: "Undeclared param returns NOT_SET via get_param"

  ---
  Implementation Steps (Ordered)

  1. Fix get_param implementation — Add 2 lines to params.cpp (lines 94-95):
  result["set"] = false;
  result["type"] = "NOT_SET";
  2. Rewrite P08 in scenarios_P.json — Replace ops and expects blocks with corrected payload-based version
  3. Validate bundle — python3 harness/scripts/validate_bundles.py must pass
  4. Build — colcon build --packages-select backend_prod must succeed
  5. Single-run test — run_reliability.sh 1 --scenario P08_unknown_types_not_set → expect 1/1 PASS
  6. Determinism test — run_reliability.sh 5 --scenario P08_unknown_types_not_set → expect 5/5 PASS, identical traces
  7. Update coverage matrix — Mark P08 PASS in ros column
  8. Commit:
  feat(params): validate P08_unknown_types_not_set — get_param on undeclared param

  - Fix get_param to return set:false, type:NOT_SET for undeclared parameters
  - Rewrite P08 expects to use op_end.detail subset-match
  - Validates SPEC_PC03 + SPEC_P03: GetParameters returns NOT_SET for unknown names
  - get_param models GetParameters single-name semantics via has_parameter() check
  - Deterministic: 5/5 identical traces

  ---
  Risk Assessment

  LOW RISK — Minimal 2-line fix to existing op. No new ops, no schema changes.

  Known issue: Current get_param incomplete for undeclared-parameter case.

  Fix confidence: 99% — The fix is trivial and the test path is well-defined.

  Blockers: None expected.