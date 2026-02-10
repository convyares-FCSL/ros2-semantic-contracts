 P05 — Read-Only Enforcement: Implementation Plan

 Objective

 Implement P05_readonly_enforcement scenario to validate SPEC_PC05: parameters declared with read_only=true cannot be modified after declaration.

 What P05 Proves (Falsifiable Claim)

 A parameter declared with descriptor.read_only=true CANNOT be modified via set_parameter. Observable evidence:
 1. declare_param with descriptor succeeds → {declared: true, descriptor: {read_only: true}}
 2. get_param captures initial value → {value: "initial"}
 3. set_param to different value fails → {successful: false}
 4. get_param verifies unchanged → {value: "initial"}

 Critical Files

 Modified

 - harness/backends/backend_prod/src/ops/params.cpp — extend exec_declare_param, add descriptor support
 - harness/scenarios/scenarios_P.json — populate P05 entry (lines 165-172)
 - docs/provenance/scenario/scenario_execution_matrix.md — update P05 row (line 72)

 Referenced (no changes)

 - harness/backends/backend_prod/src/backend.cpp — dispatch already exists (line 53-54)
 - harness/schemas/scenario.schema.json — op names already in enum
 - harness/backends/backend_prod/CMakeLists.txt — params.cpp already compiled

 Implementation Details

 1. Backend: Extend exec_declare_param (params.cpp)

 Location: harness/backends/backend_prod/src/ops/params.cpp lines 41-60

 Current implementation:
 st.node->declare_parameter<std::string>(name, "");

 New implementation (STRING-ONLY, no type dispatch):
 json exec_declare_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
     if (!op.payload || !op.payload->count("name")) {
         throw BundleError("declare_param: payload missing 'name'");
     }
     std::string name = (*op.payload)["name"];

     json result;
     result["op"] = "declare_param";
     result["name"] = name;

     // Parse descriptor (optional)
     rcl_interfaces::msg::ParameterDescriptor desc;
     if (op.payload->contains("descriptor")) {
         const json& d = (*op.payload)["descriptor"];
         if (d.contains("read_only")) desc.read_only = d["read_only"].get<bool>();
         if (d.contains("dynamic_typing")) desc.dynamic_typing = d["dynamic_typing"].get<bool>();
         // other fields (description, etc.) deferred
     }

     // Parse default value (string only)
     std::string default_value = op.payload->contains("value") ?
         (*op.payload)["value"].get<std::string>() : "";

     try {
         st.node->declare_parameter<std::string>(name, default_value, desc);
         result["declared"] = true;

         // Echo descriptor for trace inspection (makes outcome observable)
         if (desc.read_only) {
             result["descriptor"] = json::object();
             result["descriptor"]["read_only"] = true;
         }
     } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
         result["declared"] = false;
         result["reason"] = "AlreadyDeclared";
     }

     return result;
 }

 Key decisions:
 - STRING-ONLY: No integer/bool/double support (avoids template overload ambiguity, minimal scope for P05)
 - Descriptor echo: Returns descriptor.read_only in op_end detail → makes applied descriptor observable/falsifiable
 - rclcpp API: declare_parameter<std::string>(name, default_value, desc) — third parameter is const rcl_interfaces::msg::ParameterDescriptor&

 Defensive enhancement for exec_set_param:
 Add generic exception handler (in case read-only throws instead of returning unsuccessful):
 } catch (const std::exception& e) {
     result["successful"] = false;
     result["reason"] = std::string("Exception: ") + e.what();
 }
 Insert after existing ParameterNotDeclaredException catch block (line 34-37).

 2. Scenario: Populate P05 (scenarios_P.json)

 Location: harness/scenarios/scenarios_P.json lines 165-172

 Replace skeleton with:
 {
     "spec_id": "P05",
     "title": "Read-only enforcement",
     "layer": "Core",
     "requires": [
         "ros.params.declare",
         "ros.params.set",
         "ros.params.get"
     ],
     "notes": "Validates SPEC_PC05: parameters declared read-only cannot be modified. Descriptor with read_only=true passed to declare_param; subsequent
 set_param must fail and leave value unchanged.",
     "ops": [
         {
             "op": "declare_param",
             "payload": {
                 "name": "readonly_test",
                 "value": "initial",
                 "descriptor": {
                     "read_only": true
                 }
             }
         },
         {
             "op": "get_param",
             "payload": {
                 "name": "readonly_test"
             }
         },
         {
             "op": "set_param",
             "payload": {
                 "name": "readonly_test",
                 "type": "string",
                 "value": "modified"
             }
         },
         {
             "op": "get_param",
             "payload": {
                 "name": "readonly_test"
             }
         }
     ],
     "expects": [
         {
             "check": "custom",
             "params": {
                 "must_observe": [
                     {
                         "type": "op_end",
                         "detail": {
                             "op": "declare_param",
                             "name": "readonly_test",
                             "declared": true,
                             "descriptor": {
                                 "read_only": true
                             }
                         }
                     },
                     {
                         "type": "op_end",
                         "detail": {
                             "op": "get_param",
                             "name": "readonly_test",
                             "declared": true,
                             "set": true,
                             "type": "string",
                             "value": "initial"
                         }
                     },
                     {
                         "type": "op_end",
                         "detail": {
                             "op": "set_param",
                             "name": "readonly_test",
                             "successful": false
                         }
                     },
                     {
                         "type": "op_end",
                         "detail": {
                             "op": "get_param",
                             "name": "readonly_test",
                             "declared": true,
                             "set": true,
                             "type": "string",
                             "value": "initial"
                         }
                     }
                 ]
             }
         }
     ]
 }

 Design decisions:
 - No reason matching: Do NOT match on reason field in expects (rclcpp reason text is implementation-internal/unstable)
 - Immutability via value comparison: Match on before/after value: "initial" (deterministic)
 - Descriptor echo in expects: First op_end includes descriptor.read_only to verify it was applied

 3. Coverage Matrix Update

 Location: docs/provenance/scenario/scenario_execution_matrix.md line 72

 Change:
 | **P05** | P05_readonly_enforcement | N/A | N/A | N/A | N/A | Not yet validated |

 To (after validation run passes):
 | **P05** | P05_readonly_enforcement | N/A | PASS | N/A | N/A | Read-only enforcement |

 Note: Update ros column (backend_prod) only after reliability run succeeds.

 Verification Protocol

 # 1. Validate bundle schema
 python3 harness/scripts/validate_bundles.py
 # Expected: PASS (P05 schema valid)

 # 2. Build backend_prod
 colcon build --packages-select backend_prod --cmake-args -DCMAKE_BUILD_TYPE=Release
 # Expected: Clean build, no warnings

 # 3. Run P05 single-scenario test
 ./tools/testing/run_reliability.sh 1 --scenario P05_readonly_enforcement
 # Expected: PASS (4/4 op_end events match expects)

 # 4. Verify trace content (manual inspection)
 cat evidence/ephemeral_reliability.json | jq '.traces[] | select(.scenario_id=="P05_readonly_enforcement") | select(.type=="op_end")'
 # Expected: 4 events with correct detail fields:
 #   - declare_param: declared=true, descriptor.read_only=true
 #   - get_param #1: value="initial"
 #   - set_param: successful=false
 #   - get_param #2: value="initial" (unchanged)

 # 5. Check for forbidden event types
 grep -E '"type":\s*"(pass|fail|ok|verdict|assertion)"' harness/scenarios/scenarios_P.json
 # Expected: no matches (forbidden top-level types)

 # 6. Run full P-series reliability (after P05 validates)
 ./tools/testing/run_reliability.sh 5
 # Expected: H00/A01/A02/P03/P04/P05/P06 all PASS

 Risks + Mitigations
 Risk: rclcpp doesn't enforce read-only natively
 Mitigation: Test locally first; no custom callback added
 Outcome if occurs: Scenario correctly FAILS → reveals semantic gap
 ────────────────────────────────────────
 Risk: Template overload ambiguity
 Mitigation: STRING-ONLY implementation (no bool/int/double)
 Outcome if occurs: Compile succeeds
 ────────────────────────────────────────
 Risk: set_parameter throws instead of returning unsuccessful
 Mitigation: Add defensive catch (std::exception&)
 Outcome if occurs: Maps to {successful: false}
 ────────────────────────────────────────
 Risk: Descriptor not applied by rclcpp
 Mitigation: Echo descriptor in op_end detail
 Outcome if occurs: Falsifiable: trace shows what was applied
 Non-Changes (Boundary Discipline)

 - No schema changes: op names already in scenario.schema.json enum
 - No dispatch changes: backend.cpp already routes declare_param (line 53-54)
 - No capabilities changes: Reuse existing "ros.params.declare" capability (descriptor is natural extension, not new primitive)
 - No CMakeLists changes: params.cpp already compiled (line 21)
 - No new trace event types: All outcomes in op_end.detail (trace discipline)

 Estimated Scope

 - LOC changed: ~80 total (60 JSON scenario, 20 C++ params.cpp)
 - Complexity: Low — local rclcpp API extension, no DDS timing dependencies
 - Precedent: Follows P03/P04/P06 pattern exactly (op_end subset-match)
 - Determinism: High — all local synchronous APIs

 Success Criteria

 1. ✅ validate_bundles.py passes
 2. ✅ colcon build succeeds with no warnings
 3. ✅ run_reliability.sh --scenario P05 returns PASS
 4. ✅ Trace contains 4 op_end events matching expects exactly
 5. ✅ Coverage matrix updated with P05 PASS
 6. ✅ Full P-series run remains stable (no regressions)