# Parameters — Core Semantic Contract

**Normativity Class: Core Contract**

This document defines the **normative, transport-agnostic semantic contract** for parameters as enforced by a core semantics engine.
It specifies parameter truth, visibility, decision semantics, and ordering guarantees independently of any ROS transport, executor, or client library.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- parameter store state model
- value types and descriptor enforcement
- set/list/describe decision logic (outcomes)
- logical change records
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- ROS parameter services and wire protocol (see `docs/spec/global/parameters.md`)
- topic publication and QoS
- executor scheduling or threading
- lifecycle state gating
</details>

---

## Core State Model

### SPEC_PC01 — Identity and Case Sensitivity [P01]

Parameters are identified by UTF-8 string names.
- Names are **case-sensitive** and treated opaquely by the core.
- The core MUST NOT fabricate parameters that have not been explicitly set or declared.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC02 — Existence States — Derived from SPEC_PC01 [P01]

A parameter exists in exactly one of three semantic states:
1.  **Declared:** Exists with a defined type and descriptor.
2.  **Undeclared but Allowed:** Exists implicitly (only if `allow_undeclared` is true).
3.  **Unknown:** Not declared and not implicitly allowed.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

**Notes**
- Unknown parameters MUST NOT appear in list results unless explicitly requested (e.g. via recursive list).

</details>

---

## Values and Typing

### SPEC_PC03 — Type Enforcement [P02, P08, P26]

The core MUST enforce type safety based on the parameter's descriptor.
- **Static Typing (`dynamic_typing=false`):** The value type MUST NOT change after declaration. Setting an incompatible type MUST fail.
- **Dynamic Typing (`dynamic_typing=true`):** The value type MAY change on set.
- Supported types: `NOT_SET`, `BOOL`, `INTEGER`, `DOUBLE`, `STRING`, `BYTE_ARRAY`, and their array variants.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC15 — Scalar Coercion Matrix for Static Typing [P26]

Validated on measured Jazzy path (`P26_type_coercion_matrix`).

For statically typed parameters on tested scalar paths, explicit cross-type set requests are rejected rather than implicitly coerced.
- `DOUBLE -> INTEGER` set on static integer parameter: rejected.
- `BOOL -> INTEGER` set on static integer parameter: rejected.
- `INTEGER -> BOOL` set on static boolean parameter: rejected.
- Rejected set attempts MUST preserve prior value and type.

Dynamic-typed control path: explicit type transitions (`INTEGER -> DOUBLE -> INTEGER`) are accepted when `dynamic_typing=true`.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp, Jazzy+rclpy (`P26`)

**Notes**
- Scope is intentionally bounded to measured scalar paths in `P26`; array coercion and language-version edge coercions remain separate coverage work.

</details>

---

## Decision Logic (Outcomes)

### SPEC_PC04 — Declaration Logic [P03]

Declaration establishes the initial value and descriptor.
- Declaring an already-declared parameter MUST fail deterministically (outcome: `AlreadyDeclared`).
- Declaration order MUST be preserved in change records to ensure deterministic reconstruction of state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC05 — Atomic Set API Semantics [P04, P05, P18]

`set_parameters_atomically` is an all-or-nothing transaction.
- A set attempt yields a deterministic outcome: `Successful` or `Failed` (with reason).
- **Atomicity:** Failed atomic-batch operations MUST NOT partially apply. All requested changes are applied, or none.
- **Read-Only:** If `read_only=true` in the descriptor, any attempt to change the value MUST fail.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)

</details>

### SPEC_PC06 — Undeclared Access Logic [P06]

Access to undeclared parameters depends on the node's configuration.
- If `allow_undeclared_parameters` is **false**: Set attempts for unknown parameters MUST fail deterministically.
- If `allow_undeclared_parameters` is **true**: Unknown parameters become implicitly declared upon setting (type inferred from value).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp default behavior

</details>

### SPEC_PC13 — Non-Atomic Batch Apply Semantics [P17]

`set_parameters` (non-atomic batch API) applies updates per parameter and MAY partially apply.
- If a later parameter in a batch is rejected (type/read-only/callback), earlier accepted parameters MAY remain applied.
- Non-atomic batch outcomes MUST still be deterministic for each element in the submitted order.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp (`P17`)

**Notes**
- This clause distinguishes `set_parameters` from `set_parameters_atomically`.

</details>

### SPEC_PC14 — Concurrent Read Visibility Profiles [P25, P35]

Concurrent read visibility claims are probe-scoped and API-profile specific.
- For repeated single-call probes on the measured Jazzy path, mixed old/new cross-key snapshots were observed for `set_parameters` (`P25`) and not observed for `set_parameters_atomically` (`P35`).
- This clause governs externally observable interleavings under the declared probe shape, not universal scheduler behavior.
- Any future evidence of mixed visibility under stronger probes is treated as divergence evidence, not silently ignored.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (thread-interleaving semantics not normatively specified at this granularity)
- Baseline: Jazzy+rclcpp and Jazzy+rclpy (`P25`, `P35`)

**Notes**
- Measured in this repository: the previous alternating-cycle probe path was replaced by repeated single-call trials to avoid probe-induced false positives.
- The measured outcome remains a bounded observation, not a universal guarantee.

</details>

### SPEC_PC16 — Atomicity & Transactional Updates [P27, P28, P29, P30, P33, P34]

⚠️ **UNVALIDATED (baseline hypothesis)**

The core MUST distinguish between two batch update profiles:
- **Sequential/Standard (`set_parameters`)**: evaluate elements in submission order. If a later element fails validation, earlier successful elements MUST remain applied (partial application).
- **Transactional/Atomic (`set_parameters_atomically`)**: validate/apply as all-or-nothing. If any element fails validation, no element is committed.

Callback visibility contract during `on_set_parameters_callback` validation:
- Proposed new values MUST be provided in callback arguments.
- Storage reads (`get_parameter`) for a target key MUST still return the old committed value during callback execution.
- New values MUST NOT be externally observable until callback success and commit complete.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Docs: Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Baseline: Jazzy+rclcpp and Jazzy+rclpy measured set-API behavior (`P17`, `P18`), with callback visibility tracked via scenario family (`P29`, `P30`, `P33`, `P34`)

**Notes**
- This clause consolidates split semantics currently represented across `SPEC_PC05` and `SPEC_PC13` into one explicit API-profile model.
- Pinned Docker Jazzy+Humble evidence exists for `P27`-`P30`, `P33`, and `P34` across `prod` and `py`.

</details>

---

## Change Record Model

### SPEC_PC07 — Deterministic Change Records [P07]

⚠️ **UNVALIDATED (baseline hypothesis)**

The core emits logical **change records** describing modifications.
- Each operation MUST produce exactly one change record (containing one or more parameter changes).
- Ordering within a record MUST reflect the application order.
- **Determinism:** Identical operation sequences MUST produce identical records, independent of wall-clock time.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on event cardinality)
- Baseline: Jazzy+rclcpp implementation

**Notes**
- "Change Record" maps to the `ParameterEvent` message in the global scope, but at the core level, it is a logical struct.

</details>

### SPEC_PC08 — Deletion Support [P14]

⚠️ **UNVALIDATED (baseline contradicts)**

Deletion support is optional but must be deterministic if present.
- If supported, deletion of a parameter SHOULD generate a change record indicating removal.
  Baseline observation: Jazzy+rclcpp does NOT generate a change record on `undeclare_parameter`.
- Deletion of an `Unknown` parameter MUST fail or be a no-op (depending on implementation), but MUST NOT corrupt state.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- Baseline: Jazzy+rclcpp

**Notes**
- Changed from MUST to SHOULD: rclcpp baseline does not emit ParameterEvent on undeclare.
  Top-level docs are silent. Per authority ladder, rclcpp behavior defines truth.

</details>

---

## Descriptor Metadata

### SPEC_PC09 — Descriptor Reflects Current State [P21]

Validated on Jazzy baseline (`P21_descriptor_tracks_value_type`).

After a successful set operation on a dynamically typed parameter, `describe_parameters` MUST report a descriptor whose `type` field matches the current value's type.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on descriptor update semantics)
- Baseline: Jazzy+rclcpp (`P21`)

**Notes**
- Measured baseline behavior: after dynamic type change, `describe_parameters` reports descriptor `type` matching current value type.

</details>

### SPEC_PC10 — Explicit Descriptor Type Enforcement [P22]

Validated on Jazzy baseline (`P22_explicit_descriptor_type_enforcement`).

When a parameter is declared with an explicit `ParameterDescriptor.type` field, the declared type MUST be enforced.
If the initial value's type conflicts with the declared descriptor type, declaration MUST fail or the descriptor type MUST take precedence.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on descriptor.type enforcement on declare)
- Baseline: Jazzy+rclcpp (`P22`)

**Notes**
- Measured baseline behavior: explicit descriptor `type` is preserved on declaration.

</details>

### SPEC_PC17 — Descriptor Range Enforcement [P36, P37]

Validated on pinned Docker Jazzy+Humble (`P36_integer_descriptor_range_enforcement`, `P37_floating_descriptor_range_enforcement`).

If a parameter descriptor declares numeric ranges, set attempts outside those ranges MUST fail and MUST preserve the last accepted value.
- For integer-typed parameters with `integer_range`, out-of-range integer writes MUST be rejected.
- For floating-typed parameters with `floating_point_range`, out-of-range floating writes MUST be rejected.
- `describe_parameters` MUST expose the configured range metadata for the declared parameter.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 docs — Parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
- Upstream issue context: `ros2/rclcpp#728`
- Baseline: pinned Docker Jazzy+Humble, `backend_prod` + `backend_py` (`P36`, `P37`)

**Notes**
- Scope is bounded to the measured integer/floating enforcement paths in `P36` and `P37`.
- Edge cases around step granularity on non-divisible ranges are tracked separately.

</details>

### SPEC_PC11 — Declaration Without Default Value [P23, P31]

Validated on Jazzy baseline for direct declaration path (`P23_declare_without_default`), with CLI-override divergence measured in `P31_no_default_with_startup_override`.

A parameter MAY be declared without a default value if a value is expected to be supplied at launch time.
- If no value is available at declaration time, the parameter MUST be in state `NOT_SET` until set.
- Declaration without a default MUST NOT unconditionally raise an exception.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on no-default declaration behavior)
- Baseline: Jazzy+rclcpp (`P23`, `P31`)

**Notes**
- Measured baseline behavior: declaration succeeds for explicitly typed parameter with no default, and read returns `NOT_SET` until initialized.
- CLI override path measurement (`P31`): `backend_prod` passes on Jazzy/Humble, `backend_py` passes on Humble and fails on Jazzy (tracked against `rclpy#1460`).

</details>

### SPEC_PC12 — GetParameterTypes Response Contract [P24]

⚠️ **CONFORMANCE FAIL OBSERVED ON JAZZY BASELINE (`P24_get_parameter_types_response_length`)**

`get_parameter_types` MUST preserve a 1:1 request-to-response mapping.
- Response length MUST equal request length.
- Undeclared names MUST be represented as `PARAMETER_NOT_SET` entries, not collapsed into an empty vector.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: `rcl_interfaces/srv/GetParameterTypes` IDL contract
- Baseline: Jazzy+rclcpp (`P24`)

**Notes**
- Measured in this repository: both `backend_prod` and `backend_py` violate the mixed-request 1:1 expectation by returning `successful=false`, `reason=NotDeclared`, `count=0`, and `types=[]`.
- This is tracked as issue-signature coverage for `rclcpp#2512` and equivalent client-library behavior.

</details>
