This audit maps the **Semantic Scenarios** (A, G, H, L, P, S) to their specifications and evaluates their utility. This reveals where the test suite is enforcing critical ROS 2 physics versus where it is merely calibrating to implementation quirks ("Rot").

---

### Action Scenario Semantics (A)

| ID | Title | Validates | Layer | Usefulness | Status |
| --- | --- | --- | --- | --- | --- |
| **A01** | No Ghosts After Rejection | `SPEC_AC01` | Core | **Critical**: Fundamental object permanency. | Required |
| **A02** | Terminal State Immutability | `SPEC_AC03` | Core | **Critical**: Safety invariant. | Required |
| **A03** | Unknown Goal Not Persistent | `SPEC_AC01` | Core | **High**: Graph hygiene. | Required |
| **A04** | Valid Transitions Only | `SPEC_AC04` | Core | **Critical**: State machine integrity. | Required |
| **A05** | Invalid Transitions Rejected | `SPEC_AC04` | Core | **Critical**: State machine integrity. | Required |
| **A06** | Status Updates Monotonic | `SPEC_AC05` | Core | **High**: Client logic stability. | Required |
| **A07** | Ordering by Sequence | `SPEC_AC05` | Core | **High**: Determinism over transport. | Required |
| **A08** | Cancellation Intent Observable | `SPEC_AC06` | Core | **Critical**: Control surface visibility. | Required |
| **A09** | Cancel Resolution Policy | `SPEC_AC06` | Core | **High**: Defines cancellation contract. | Required |
| **A10** | Result Visible After Terminal | `SPEC_AC07` | Core | **High**: Late-joiner support. | Required |
| **A11** | Single Terminal Result | `SPEC_AC07` | Core | **High**: Protocol correctness. | Required |
| **A12** | Supersession via CANCELED | `SPEC_A03` | Policy | **Low**: Enforces `rclcpp` convention over physics. | **Rot Candidate** |
| **A13** | Result Retention Non-Zero | `SPEC_A05` | Policy | **High**: Practical usability. | Required |
| **A14** | Tooling Compatibility | `SPEC_A07` | Tooling | **High**: Ecosystem interoperability. | Required |
| **A15** | No Goal ID Reuse | `SPEC_AC02` | Core | **Critical**: Safety against replay attacks. | Deferred |
| **A16** | Action Interface Conformity | `SPEC_A01` | Global | **Critical**: Topology validation. | Required |
| **A17** | Abandoned Goal Cleanup | `SPEC_SYS06` | System | **High**: Long-running stability. | Should Have |

---

### Lifecycle Scenario Semantics (L) & Global (G)

| ID | Title | Validates | Layer | Usefulness | Status |
| --- | --- | --- | --- | --- | --- |
| **L01** | Only Defined Transitions | `SPEC_LC02` | Core | **Critical**: State machine integrity. | Required |
| **L02** | Primary State Visibility | `SPEC_L02` | Core | **High**: Tooling stability. | Required |
| **L03** | Intermediate States Hidden | `SPEC_LC03` | Core | **High**: Atomicity invariant. | Required |
| **L04** | Single Active Transition (Lock) | `SPEC_LC04` | Core | **Critical**: Concurrency safety. | Required |
| **L05** | Concurrent Transition Rejected | `SPEC_LC04` | Core | **Critical**: Concurrency safety. | Required |
| **L06** | ErrorProcessing Deterministic | `SPEC_LC05` | Core | **Low**: Enforces `rclcpp` error mapping. | **Rot Candidate** |
| **L07** | GetState Reports Primary Only | `SPEC_L02` | Core | **High**: Protocol correctness. | Required |
| **L08** | Rejection Observable | `SPEC_L03` | Core | **High**: Feedback loop integrity. | Required |
| **L09** | TransitionEvent Emitted | `SPEC_L04` | Core | **Low**: High noise, implementation detail. | **Bloat?** |
| **L10** | Shutdown Deterministic | `SPEC_L05` | Core | **Critical**: Cleanup guarantee. | Required |
| **L11** | Shutdown via Deactivate | `SPEC_L06` | Core | **High**: Safe teardown sequence. | Should Have |
| **L12** | Services Not Gated | `SPEC_L08` | Core | **Med**: Clarifies availability model. | Should Have |
| **L13** | Lifecycle Interface Conformity | `SPEC_L01` | Global | **Critical**: Topology validation. | Required |
| **G01** | Inactive Suppresses Pubs | `SPEC_L07` | Core | **High**: Lifecycle functional contract. | Required |

---

### Parameter Scenario Semantics (P)

| ID | Title | Validates | Layer | Usefulness | Status |
| --- | --- | --- | --- | --- | --- |
| **P01** | No Fabricated Parameters | `SPEC_PC01` | Core | **High**: State hygiene. | Required |
| **P02** | Dynamic Typing Rules | `SPEC_PC03` | Core | **Critical**: Type safety. | Required |
| **P03** | Redeclaration Fails | `SPEC_PC04` | Core | **High**: State machine integrity. | Required |
| **P04** | No Partial Application | `SPEC_PC05` | Core | **Critical**: Atomic Transaction Physics. | Required |
| **P05** | Read-Only Enforcement | `SPEC_PC05` | Core | **High**: Configuration contract. | Required |
| **P06** | Unknown Parameters Rejected | `SPEC_PC06` | Core | **High**: Security/Config safety. | Required |
| **P07** | Change Record Ordering | `SPEC_PC07` | Core | **Low**: No upstream spec. | **Bloat?** |
| **P08** | Unknown Types NOT_SET | `SPEC_PC03` | Core | **High**: Protocol correctness. | Required |
| **P09** | Atomic Updates (External) | `SPEC_P04` | Core | **High**: Atomicity visibility. | Required |
| **P10** | Rejected Updates Silent | `SPEC_P04` | Core | **High**: Event stream hygiene. | Required |
| **P11** | Events Describe Changes | `SPEC_P05` | Core | **High**: Observer utility. | Required |
| **P12** | Describe Unknown Consistent | `SPEC_P06` | Core | **Low**: Undefined behavior calibration. | **Rot Candidate** |
| **P13** | List Complete and Stable | `SPEC_P07` | Tooling | **Med**: Tooling utility. | Should Have |
| **P14** | Deletion Reported via Events | `SPEC_PC08` | Policy | **Low**: Downgraded for `rclcpp` bug. | **Rot Candidate** |
| **P15** | Parameter Interface Conformity | `SPEC_P01` | Global | **Critical**: Topology validation. | Required |
| **P16** | Explicit Declaration Mode | `SPEC_P02` | Core | **High**: Config contract. | Required |
| **P17** | Non-Atomic Batch Callback | `SPEC_PC13` | Global | **High**: Differentiates batch types. | Required |
| **P18** | Atomic Batch Callback | `SPEC_PC05` | Global | **High**: Differentiates batch types. | Required |
| **P19** | Pre-Set Callback Removal | Discovery | Global | **Med**: Advanced usage pattern. | Should Have |
| **P20** | Event Cardinality | Issue Probe | Global | **Low**: Measures implementation jitter. | **Bloat** |
| **P21** | Descriptor Tracks Dynamic | `SPEC_PC09` | Core | **High**: UI synchronization. | Should Have |
| **P22** | Explicit Descriptor Type | `SPEC_PC10` | Core | **High**: Declaration intent. | Should Have |
| **P23** | Decl Without Default | `SPEC_PC11` | Core | **High**: Production pattern. | Should Have |
| **P24** | GetTypes 1:1 Map | `SPEC_PC12` | Core | **Critical**: IDL Conformance. | **Required (Failing)** |
| **P25** | Concurrent Read (Non-Atomic) | `SPEC_PC14` | Core | **Med**: Performance envelope. | Should Have |
| **P26** | Coercion Matrix | `SPEC_PC15` | Core | **High**: Data integrity. | Required |
| **P27** | Standard Batch Partial | `SPEC_PC13` | Core | **High**: Differentiates batch types. | Required |
| **P28** | Atomic Batch Transaction | `SPEC_PC05` | Core | **Critical**: Transaction integrity. | Required |
| **P29** | Callback Storage Old-Value | `SPEC_PC16` | Core | **High**: Validation logic safety. | Should Have |
| **P30** | Ext Visibility Success Only | `SPEC_PC16` | Global | **High**: Observer safety. | Should Have |
| **P31** | No-Default w/ CLI Startup Override | `SPEC_PC11` | Core | **High**: Production pattern. | Should Have |
| **P32** | Startup Event Batching | `SPEC_P05` | Global | **Low**: Measures implementation jitter. | **Bloat** |
| **P33** | Std Batch Partial (Type) | `SPEC_PC16` | Core | **High**: Differentiates batch types. | Should Have |
| **P34** | Atomic Batch Integrity | `SPEC_PC16` | Core | **Critical**: Transaction integrity. | Should Have |
| **P35** | Concurrent Read (Atomic) | `SPEC_PC14` | Core | **Med**: Performance envelope. | Should Have |
| **P36** | Integer Descriptor Range Enforcement | `SPEC_PC17` | Core | **High**: Enforces descriptor numeric bounds for integer parameters. | Required |
| **P37** | Floating Descriptor Range Enforcement | `SPEC_PC17` | Core | **High**: Enforces descriptor numeric bounds for floating parameters. | Required |

---

### System Scenario Semantics (S) & Harness (H)

| ID | Title | Validates | Layer | Usefulness | Status |
| --- | --- | --- | --- | --- | --- |
| **S01** | Load Node Observable | `SPEC_C02` | System | **Critical**: Orchestration basis. | Required |
| **S02** | Load Failure Clean | `SPEC_C02` | System | **High**: Error recovery. | Required |
| **S03** | Unload Node Destroyed | `SPEC_C03` | System | **Critical**: Resource cleanup. | Required |
| **S04** | Unique Node IDs | `SPEC_C04` | System | **High**: Graph safety. | Required |
| **S05** | Concurrent Load Deterministic | `SPEC_C05` | System | **Low**: Measures busy-loop logic. | **Bloat?** |
| **S06** | Remaps Apply at Load | `SPEC_C06` | System | **High**: Configuration contract. | Required |
| **S07** | Explicit Ownership | `SPEC_E01` | System | **Critical**: No hidden threads. | Required |
| **S08** | Bounded Spin Modes | `SPEC_E02` | System | **High**: Testability. | Required |
| **S09** | Shutdown Bounded | `SPEC_E03` | System | **Critical**: Termination guarantee. | Required |
| **S10** | Callback Failure Visible | `SPEC_E05` | System | **High**: Failure observability. | Required |
| **S11** | Lifecycle-Executor Safety | `SPEC_E06` | System | **Med**: Composition safety. | Should Have |
| **S12** | Lifecycle Orchestration | `SPEC_ECO01` | System | **High**: Nav2 compatibility. | Should Have |
| **S13** | Liveness Detection | `SPEC_ECO02` | System | **High**: System robustness. | Deferred |
| **S14** | Tooling Interoperability | `SPEC_ECO04` | System | **High**: Developer experience. | Required |
| **S15** | Explicit Ownership/Lifetime | `SPEC_SYS01` | System | **High**: Architecture hygiene. | Required |
| **S16** | Deterministic Bringup | `SPEC_SYS02` | System | **High**: Orchestration basis. | Required |
| **S17** | No Hidden Execution | `SPEC_SYS03` | System | **High**: Real-time hygiene. | Deferred |
| **S18** | Observable Failure | `SPEC_SYS04` | System | **High**: Operational hygiene. | Required |
| **S19** | Graph Hygiene | `SPEC_SYS05` | System | **Low**: Tests DDS race conditions. | **Rot Candidate** |
| **S20** | Composition Conformity | `SPEC_C01` | System | **Critical**: Topology validation. | Required |
| **H00** | Behavioral Indistinguishability | `SPEC_ECO05` | System | **Meta**: The Harness Contract. | Required |
