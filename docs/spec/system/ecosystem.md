# Ecosystem Contract

**Normativity Class: Baseline System Contract**

These requirements define interoperability constraints for the ROS 2 professional baseline.
They are validated via system-level oracle and harness tests, not core semantic engines.

* **Authority:** Production Stack (Jazzy + rclcpp).
* **Evidence:** Nav2 usage patterns provide high-leverage evidence for hidden ecosystem invariants.
* **Status:** Until validated by traces, requirements marked UNVALIDATED remain hypotheses.

---

<details open>
<summary><strong>Scope: What this contract covers</strong></summary>

- native ROS 2 client library implementations built on `rcl` / `rmw`
- interoperability with ROS 2 CLI tooling and lifecycle managers
- compatibility with mature ecosystem stacks (e.g. Navigation 2)
</details>

<details>
<summary><strong>Scope: What this contract excludes</strong></summary>

- specific implementation strategies or language choices
- adoption of ecosystem libraries beyond semantic compatibility
- non-native transports (unless explicitly stated)
</details>

---

## Ecosystem Invariants

### SPEC_ECO01 — Lifecycle Orchestration Compatibility [S12]

An implementation MUST expose lifecycle behaviour compatible with external lifecycle managers to ensure safe system orchestration.
- Lifecycle state, transitions, and failure responses MUST be observable and controllable via standard ROS 2 lifecycle services.
- An implementation that cannot be externally orchestrated via these standard interfaces is non-compliant.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: [ROS 2 Design: Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- Community: Nav2 (manager expectations)
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Orchestration requirement)

</details>

### SPEC_ECO02 — Liveness and Fault Detection [S13]

Active components MUST provide a mechanism for liveness detection to ensure system robustness.
- A crashed or unresponsive component MUST be detectable by external observers.
- Orchestration layers MUST be able to react deterministically to a loss of liveness.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on specific liveness guarantees)
- Community: Nav2 (Bond-based monitoring)
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Reliability requirement)

**Notes**
- The specific mechanism (e.g., bonds) is ecosystem-defined, but the **guarantee** that a crash is observable is normative.

**Automation note — deferred**
- Exercising this spec requires a kill-node or adversarial timeout primitive not available in the current ops model. Scenario S13 declares the claim but cannot be deterministically triggered until such a primitive is added.

</details>

### SPEC_ECO03 — Action Supersession Semantics [A12]

⚠️ **UNVALIDATED (baseline hypothesis)**

When a new goal supersedes an existing active goal (preemption), the server must handle the transition deterministically:
- The previously active goal MUST transition to a terminal state indicating supersession.
- In the baseline profile, supersession MUST be represented as `CANCELED`, with result or reason information indicating preemption.
- Supersession MUST NOT be represented as `ABORTED` (which implies internal malfunction).

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None (upstream silent on specific preemption state)
- Community: Nav2 (Safety requirement for continuous replanning)
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Semantic distinction between failure and preemption)

**Notes**
- This constraint reflects ecosystem expectations for safe behavior in long-running action servers.


</details>

### SPEC_ECO04 — Tooling Introspection Support [S14]

A compliant implementation MUST support ecosystem tooling expectations to ensure observability.
- **Action Discovery:** Action servers must be discoverable by CLI tools.
- **Parameter Events:** Changes to parameters must be published to `/parameter_events`.
- **Lifecycle Introspection:** Current state must be queryable via `get_state`.

<details>
<summary>Sources and notes</summary>

**Sources**
- Official: None
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (Tooling observability)

**Notes**
- These behaviors are validated via oracle and harness testing, not by assertion alone.

</details>

### SPEC_ECO05 — Behavioral Indistinguishability [H00]

An implementation MUST behave **indistinguishably** from other compliant implementations when observed through standard ROS 2 tools.
- Semantic compatibility is defined strictly by **observable behaviour** (wire protocol, graph state, service responses), not internal APIs.

<details>
<summary>Sources and notes</summary>

**Sources**
- BIC: [system_contract.md#baseline-interoperability-constraints](system_contract.md) (The fundamental interoperability axiom)

</details>