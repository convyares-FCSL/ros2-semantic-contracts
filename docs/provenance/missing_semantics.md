## Brief overview

This is exactly the right document to write **now**.
It should be mechanical, boring, and brutally explicit: a punch-list for the oracle harness, not an essay.

Below is a **filled-in, baseline-aligned version** that reflects everything you’ve already marked ⚠️ UNVALIDATED across lifecycle, actions, and parameters — with no new semantics introduced.

---

# Missing Semantics Inventory

This document enumerates **semantic gaps** where upstream ROS 2 specifications are incomplete and where behaviour must be established via:

* implementation evidence (rclcpp),
* ecosystem evidence (Nav2),
* and/or reproducible oracle testing against the baseline stack.

Items listed here are **not yet normative**.
They must be validated against the baseline profile (**ROS 2 Jazzy + rclcpp + Nav2**) before promotion to enforced contract.

---

## Lifecycle

### Transition completion semantics

* Whether `ChangeState` service calls block until transition completion or return after acceptance.
* Observable behaviour under long-running `on_*` callbacks.
* Requires oracle measurement of Jazzy `rclcpp_lifecycle` service behaviour.

### TransitionEvent emission rules

* Whether busy or rejected transitions emit `transition_event` messages.
* Ordering guarantees between state change completion and event publication.
* Requires oracle observation under contention and invalid transition attempts.

### ErrorProcessing default resolution

* Default mapping of `on_error` return values to terminal states in Jazzy `rclcpp_lifecycle`.
* Hypothesis: default FAILURE → Finalized.
* Marked ⚠️ UNVALIDATED until oracle confirms.

### Lifecycle-managed publisher gating

* Confirmation that lifecycle-managed publishers emit **no messages** while Inactive.
* Scope strictly limited to lifecycle-managed publishers (no timers, subs, actions).
* Requires harness that attempts publication while Inactive.

---

## Actions

### Supersession terminal representation

* How goal supersession is represented in practice in the baseline stack.
* Hypothesis: supersession is represented as `CANCELED` with a reason indicating preemption.
* Must verify against Nav2 action servers and `rclcpp_action`.

### Result / status retention duration

* Whether the baseline retains terminal results/status for a non-zero duration.
* Exact duration is implementation-defined, but non-zero retention is a possible baseline expectation.
* Requires delayed `GetResult` oracle tests.

### Feedback / status ordering guarantees

* Validation that ordering is governed by per-goal sequence fields, not wall-clock timing.
* Ensure no observable regressions under concurrent feedback and status publication.

### Cancel race resolution envelope

* Confirmation that terminal outcomes while Canceling are limited to `{Canceled, Succeeded, Aborted}`.
* Ensure no illegal or intermediate terminal states are observable.

---

## Parameters

### ParameterEvent cardinality

* Whether each accepted atomic update produces exactly one `ParameterEvent`.
* Hypothesis: one event per accepted update batch.
* Requires oracle validation under multi-parameter updates.

### ListParameters prefix/depth semantics

* Behaviour of `prefixes` and `depth` filtering in Jazzy `rclcpp`.
* Only stability and completeness are currently normative.
* Detailed filtering semantics remain ⚠️ UNVALIDATED.

### Atomic visibility to observers

* Confirmation that no external observer (service call or event stream) can observe a partially applied update.
* Expected invariant but requires oracle stress testing.

---

## Executors / Composition / Ecosystem

### Executor fairness characteristics

* Confirmation that executor scheduling does not violate stated observable guarantees (e.g. starvation).
* Not asserting fairness — only detecting pathological divergence.

### Composition bringup observability timing

* Validation that nodes become visible on the ROS graph within an implementation-defined bounded time.
* Detect non-termination or silent partial bringup.

### Liveness detection mechanisms

* Confirmation that baseline components provide a detectable liveness signal while Active.
* Mechanism (bond or equivalent) is not prescribed, only the guarantee.

---

## Promotion Rule

An item may be removed from this inventory and promoted to a **normative contract** only when:

1. Oracle tests reproduce the behaviour against the baseline stack, and
2. The behaviour is stable, observable, and relied upon by the ecosystem.

Until then, all items listed here remain **explicitly non-normative**.
