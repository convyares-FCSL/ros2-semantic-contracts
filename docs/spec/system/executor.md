# Executor — System Contract

This document defines **system-level semantic constraints** for executor behaviour
required to interoperate with the ROS 2 professional baseline.

These constraints are **not core semantic engine invariants**.
They describe observable execution, shutdown, and concurrency behaviour
that must hold at the system boundary.

Baseline profile:
- ROS 2 Jazzy + rclcpp + Nav2

---

## Scope

This contract applies to:
- executor ownership and spinning
- bounded shutdown and cancellation
- callback concurrency guarantees
- lifecycle interaction under load
- testability of execution behaviour

This contract does **not** define:
- client-library APIs
- callback scheduling algorithms
- real-time guarantees
- QoS tuning
- component composition mechanics

---

## Executor ownership (explicit)

The system MUST make executor ownership explicit.

Observable requirements:
- No callbacks are **invoked** unless an executor is actively being spun.
- Libraries MUST NOT start background spinning implicitly.
- If background execution exists, it MUST be explicitly constructed,
  documented, and controllable.

Hidden or implicit execution is non-compliant.

---

## Spin modes

The system MUST support bounded execution modes sufficient for
deterministic integration and testing:

- spin-forever (block until shutdown)
- spin-once / step (process a bounded unit of work)
- spin-with-timeout

Exact API shape is implementation-defined.

---

## Shutdown semantics

Shutdown MUST be bounded and deterministic:

- A shutdown request MUST unblock any active spin loop within a bounded time.
- Shutdown MUST be idempotent.
- Dropping executor or node handles MUST NOT deadlock.

Observable requirement:
- A process exits cleanly without repeated external interruption.

---

## Callback concurrency model

The system MUST define and document its concurrency model.
This contract does **not** mandate a particular concurrency model.

Observable considerations:
- Whether callbacks may execute concurrently
- Whether per-entity ordering is preserved
- Whether starvation is possible

Baseline expectations (informative, not normative):
- Single-threaded execution → no concurrent callbacks
- Multi-threaded execution → concurrency permitted; ordering best-effort

---

## Callback isolation and failure handling

Callback failures MUST NOT silently corrupt execution:

- Failure handling policy MUST be documented.
- If a failure terminates execution, it MUST be explicit and observable
  (e.g. via process exit, error logging, or lifecycle transition).
- Long-running callbacks MUST NOT permanently block shutdown.

---

## Lifecycle interaction

Executor and lifecycle semantics MUST compose safely:

- Lifecycle transitions MUST NOT require re-entrant spinning of the same executor.
- Transition handling MUST NOT deadlock under executor load.
- Busy rejection semantics MUST remain deterministic even under contention.

---

## Testing support (harness expectation)

Executor behaviour MUST be testable via harness:

- bounded spin/step capability
- deterministic shutdown
- reproducible lifecycle + executor interaction

Tooling and test validation is performed via oracle and harnesses,
not asserted by this document alone.

---

## Provenance

### Upstream sources
- TBD

### Implementation-defined (rclcpp)
- TBD

### Ecosystem-defined (Nav2)
- TBD

### Project policy
- Baseline executor operability requirements
