# Boundary Contract — SUT / Peer Separation

**Authority:** Normative. Single source of truth for component-role
assignments in all scenario families.
**Scope:** All families. Machine-enforced scope: Actions (Axx) initially.
**Referenced from:** `docs/CONTEXT.md`, `CLAUDE.md`

---

## 1. Definitions

### SUT (System Under Test)

The component whose semantic behaviour the scenario exists to validate.
Only interactions exercised through real client-library APIs on the SUT
side produce normatively meaningful evidence.

### Peer

The counterpart that provides stimuli to — or receives outputs from — the
SUT.  The peer is harness apparatus, not a validation target.  Its
behaviour must be independently determined; it must not be scripted to
produce the specific outcome the scenario checks for.

---

## 2. The Hard Rule

> A backend must never host a policy-driven peer whose decision logic is
> scripted — directly or indirectly — to produce the outcome that the
> scenario's `expects` block checks for.

This pattern is a **self-confirming peer**.  It makes a scenario
unfalsifiable: the peer cannot surprise the SUT, so semantic divergence
across client libraries cannot be detected.

A peer is self-confirming when **all** of the following hold:

1. It resides in the same backend process, or is spawned / configured by
   that process.
2. Its accept / reject / cancel decisions are determined by scenario-
   supplied parameters (e.g. `ops[].payload.mode`).
3. Those decisions are the stimulus that the scenario's `expects` block
   is checking.

**Important:** using real client-library APIs does not exempt a peer from
this rule.  The APIs may be genuine `rclcpp` calls; what matters is
whether the *decision logic* is scenario-authored.  This is the gap that
this contract closes relative to the existing "Strict Fidelity" gate in
`docs/harness/RELIABILITY_RULES.md` §No-Cheating, which governs API
realism but not decision autonomy.

---

## 3. Rule: Actions (Axx)

For every scenario whose ID matches `A\d{2}_*`:

| Role | Assignment | Rationale |
|------|------------|-----------|
| SUT  | Action **client** | Axx validates client-side goal lifecycle semantics. |
| Peer | Action **server** | The server decides accept / reject / cancel; it is the stimulus source. |

Enforcement requirements:

1. `boundary.sut` MUST be `"client"`.
2. `boundary.peer` MUST be `"external"` — the action server must be
   provided by the scenario environment (separate process or container),
   not authored or policy-configured by the backend.
3. Evidence must originate from real ROS 2 client-library interactions
   between the SUT and the external peer.  Backend ops may *communicate a
   desired stimulus* to the environment (e.g. "please reject this goal"),
   but the decision and its execution must happen in the external peer,
   not in backend-local code.

---

## 4. Other Families

Lifecycle (Lxx), Parameter (Pxx), and other families may allow self-hosted
components when that component is itself the SUT (e.g. a lifecycle node
whose transitions are being validated).  Boundary rules for those families
are out of scope here and will be defined in future editions of this
document.

---

## 5. Relationship to Existing Constraints

| Constraint | What it governs | What it does NOT govern |
|---|---|---|
| `RELIABILITY_RULES.md` §Strict Fidelity | API realism (real `rclcpp` calls) | Peer decision autonomy |
| `CLAUDE.md` §Non-negotiables | No simulated behaviour | Scope of "simulated" w.r.t. policy-driven peers |
| **This document** | Peer decision autonomy | — |
