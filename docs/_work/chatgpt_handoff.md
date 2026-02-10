# ChatGPT Handoff: Phase 2 (Backend Coding) → Phase 3 (Measure) → Phase 4 (Document)

## Context

You are working on the `ros2-semantic-contracts` project at `~/ai-workspace/projects/ros2-semantic-contracts`. This project codifies observable ROS 2 behavior into testable contracts by running scenarios against `backend_prod` (rclcpp/C++) and `backend_py` (rclpy/Python) inside Docker.

**Authority model:** The only authority is observable results from rclcpp+Jazzy and the open codebase. Nothing speculative. Every claim must be backed by a reproducible trace.

### What has been completed

1. **Phase 0 (Evaluator Audit):** The evaluator's `matches_pattern` + `must_observe` consume+order semantics are sound. No false-pass risk found. No changes needed.

2. **Phase 1 (Specs):** Four new UNVALIDATED clauses added to `docs/spec/core/parameter_core.md`:
   - SPEC_PC09: Descriptor reflects current state after set
   - SPEC_PC10: Explicit ParameterDescriptor.type enforcement
   - SPEC_PC11: Declaration without default value
   - SPEC_PC12: GetParameterTypes response contract
   - SPEC_P08/PC08 fixed: deletion events MUST→SHOULD (matches rclcpp baseline)

3. **Phase 2 (Scenarios — JSON only):** P21-P24 added to `harness/scenarios/scenarios_P.json`. P14 notes updated.

### What remains — YOUR TASK

Complete the remaining Phase 2 coding, then Phase 3 and Phase 4:

---

## Phase 2 Remaining: Backend Implementation

### P21 — No code changes needed
Uses existing ops: `declare_param` (with `dynamic_typing: true`), `set_param`, `describe_param`. Should run as-is.

### P22 — Enhance `declare_param` to accept `descriptor.type`

**Files:**
- `harness/backends/backend_prod/src/ops/params.cpp`: In the `exec_declare_param` function, the descriptor parsing block (around line 145) already handles `read_only` and `dynamic_typing`. Add parsing for `descriptor.type` — read the integer from payload, set `descriptor.type = static_cast<uint8_t>(type_val)`.
- `harness/backends/backend_py/backend_py/ops/params.py`: Same — in the `declare_param` handler, parse `descriptor.type` from payload and pass to `ParameterDescriptor(type_=type_val)`.

### P23 — Enhance `declare_param` for no-default mode

**Files:**
- `harness/backends/backend_prod/src/ops/params.cpp`: Allow `value` to be absent. If no `value` field in payload, call `node->declare_parameter(name, rclcpp::ParameterType::PARAMETER_NOT_SET, descriptor)` or equivalent no-default overload.
- `harness/backends/backend_py/backend_py/ops/params.py`: Same — if no `value`, call `node.declare_parameter(name, descriptor=descriptor)` without a value argument.

### P24 — New op: `get_parameter_types`

**Files:**
- `harness/backends/backend_prod/src/ops/params.cpp`: New function `exec_get_parameter_types()`:
  ```cpp
  json exec_get_parameter_types(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
      auto names = op.payload->at("names").get<std::vector<std::string>>();
      auto types = st.node->get_parameter_types(names);
      json result;
      result["op"] = "get_parameter_types";
      result["count"] = types.size();
      json type_arr = json::array();
      for (auto t : types) {
          type_arr.push_back(static_cast<int>(t));
      }
      result["types"] = type_arr;
      return result;
  }
  ```
- `harness/backends/backend_prod/src/backend.cpp`: Add to dispatch: `else if (op.op == "get_parameter_types") { op_result = ops::exec_get_parameter_types(w, st, scenario_id, op); }`
- `harness/backends/backend_py/backend_py/ops/params.py`: New handler:
  ```python
  def op_get_parameter_types(w, st, scenario_id, op):
      names = op['payload']['names']
      types = st.node.get_parameter_types(names)
      return {
          'op': 'get_parameter_types',
          'count': len(types),
          'types': [int(t) for t in types]
      }
  ```
- `harness/backends/backend_py/backend_py/backend.py`: Add to `OP_HANDLERS`: `'get_parameter_types': params.op_get_parameter_types`

### P25 — New op: `concurrent_read_during_batch` + scenario JSON

**C++ (`backend_prod/src/ops/params.cpp`):**
```cpp
json exec_concurrent_read_during_batch(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    auto target_params = op.payload->at("target_params").get<std::vector<std::string>>();
    auto batch = op.payload->at("batch");  // array of {name, type, value}
    
    std::atomic<bool> done{false};
    std::vector<json> snapshots;
    std::mutex snap_mutex;
    
    // Reader thread: poll parameters in tight loop
    std::thread reader([&]() {
        while (!done.load()) {
            json snap;
            for (const auto& name : target_params) {
                auto param = st.node->get_parameter(name);
                snap[name] = param.value_to_string();
            }
            std::lock_guard<std::mutex> lock(snap_mutex);
            snapshots.push_back(snap);
        }
    });
    
    // Main thread: fire non-atomic set
    std::vector<rclcpp::Parameter> params;
    for (const auto& p : batch) {
        params.push_back(make_parameter(p));  // use existing helper
    }
    auto results = st.node->set_parameters(params);
    
    done.store(true);
    reader.join();
    
    // Analyze: did the reader ever see a partial state?
    bool partial_observed = false;
    // ... analyze snapshots for intermediate states where some params
    // have old values and some have new values
    
    json result;
    result["op"] = "concurrent_read_during_batch";
    result["partial_state_observed"] = partial_observed;
    result["snapshot_count"] = snapshots.size();
    return result;
}
```

**Python (`backend_py/ops/params.py`):** Same logic with `threading.Thread`.

**Scenario JSON** to add to `scenarios_P.json`:
```json
"P25_concurrent_visibility_probe": {
    "spec_id": "P25",
    "title": "Concurrent reader detects partial state during non-atomic batch",
    "layer": "Core",
    "requires": ["ros.params.declare", "ros.params.concurrent_read"],
    "notes": "Validates SPEC_PC05 extension: measures whether a concurrent reader can observe partial state during set_parameters (non-atomic). rclcpp uses mutex (no partial state expected). rclpy has no lock (partial state may be observable).",
    "ops": [
        {"op": "declare_param", "payload": {"name": "p25_a", "value": "old_a", "descriptor": {"dynamic_typing": true}}},
        {"op": "declare_param", "payload": {"name": "p25_b", "value": "old_b", "descriptor": {"dynamic_typing": true}}},
        {"op": "concurrent_read_during_batch", "payload": {
            "target_params": ["p25_a", "p25_b"],
            "batch": [
                {"name": "p25_a", "type": "string", "value": "new_a"},
                {"name": "p25_b", "type": "string", "value": "new_b"}
            ]
        }}
    ],
    "expects": [{
        "check": "custom",
        "params": {
            "must_observe": [
                {"type": "op_end", "detail": {"op": "concurrent_read_during_batch", "partial_state_observed": false}}
            ]
        }
    }]
}
```

---

## Phase 3: Measure

After all ops compile and scenarios run without crash:

### Step 1: Build
```bash
cd ~/ai-workspace/projects/ros2-semantic-contracts
# Docker build for both backends
docker compose build backend_prod backend_py
```

### Step 2: Verify against rclcpp baseline
```bash
# Run P21-P25 on rclcpp
docker compose run --rm backend_prod scenarios_P 2>&1 | grep -E "P2[1-5]"
```
- If PASS: spec hypothesis confirmed
- If FAIL: **the spec hypothesis was wrong** — correct the spec clause in `docs/spec/core/parameter_core.md` to match what rclcpp actually does, then re-derive the scenario expectations

### Step 3: Run against rclpy
```bash
docker compose run --rm backend_py scenarios_P 2>&1 | grep -E "P2[1-5]"
```
Record which scenarios PASS and which FAIL.

### Step 4: Regression
```bash
# Confirm P01-P20 still pass for both
docker compose run --rm backend_prod scenarios_P
docker compose run --rm backend_py scenarios_P
```

---

## Phase 4: Document

### Update scenario_execution_matrix.md
File: `docs/provenance/scenario/scenario_execution_matrix.md`

Add rows for P21-P25 with observed PASS/FAIL.

### Update missing_semantics.md
File: `docs/provenance/missing_semantics.md`

Remove any hypotheses now validated. Add cross-references to new scenarios.

### File divergence records
For each scenario that PASSES on rclcpp but FAILS on rclpy, create a divergence record in `docs/provenance/divergence_records/` following the format in `docs/provenance/divergence_records/README.md`.

Each record must include:
- Scenario ID and spec reference
- Exact Docker commands to reproduce
- Raw trace excerpts showing the divergent behavior
- Classification: Defect, Gap, Fork, or Baseline Regression

> **IMPORTANT**: Create initial divergence records with the information above, but do NOT tie them to GitHub issues. Leave a `## Upstream References` section in each record with `TBD — to be linked during review`. A separate reviewer will cross-reference these against the known GitHub issues and add the links.

---

## Review Handback

When you are done with Phase 3 and Phase 4, leave a summary in `docs/_work/chatgpt_results.md` containing:
1. Per-scenario results table (P21-P25 × backend_prod/backend_py)
2. List of divergence records created (filenames)
3. Any spec hypotheses that needed correction (with what you changed)
4. Any scenarios that couldn't run (missing ops, crashes, etc.)
5. Regression results for P01-P20

This file will be reviewed by the project lead, who will:
- Verify the divergence records
- Tie them to the corresponding GitHub issues (rclcpp#2512, rclpy#1460, rclcpp#728, etc.)
- Promote or correct the UNVALIDATED spec clauses based on the evidence

---

## Key files to read first

| File | Purpose |
|---|---|
| `docs/_work/implementation_plan.md` | Full detail on specs, scenario shapes, file changes |
| `docs/_work/task.md` | Checklist with completed/remaining items |
| `docs/_work/plan.md` | Concise plan overview |
| `harness/backends/backend_prod/src/ops/params.cpp` | C++ parameter ops — add new ops here |
| `harness/backends/backend_prod/src/backend.cpp` | C++ op dispatch — register new ops |
| `harness/backends/backend_py/backend_py/ops/params.py` | Python parameter ops — add new handlers |
| `harness/backends/backend_py/backend_py/backend.py` | Python op dispatch — register handlers |
| `harness/scenarios/scenarios_P.json` | Scenario definitions — P21-P24 already added |
| `docs/spec/core/parameter_core.md` | Spec clauses — PC09-PC12 already added |

## Critical constraints

1. **No speculative claims.** Every spec clause is a hypothesis until rclcpp proves it. If rclcpp doesn't behave as hypothesized, fix the spec to match reality.
2. **Minimal changes.** Only add what's needed to produce evidence. No framework extensions, no refactors.
3. **Authority is traces.** The raw trace JSONL is the evidence. Divergence records must include reproduction commands and trace excerpts.
4. **Follow existing patterns.** Look at how existing ops (`set_param`, `describe_param`, etc.) are implemented. Match the style exactly.
