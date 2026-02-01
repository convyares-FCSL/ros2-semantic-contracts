# Oracle Baseline — ROS 2 Jazzy Professional Stack

This document defines the **[R1] Authority**: the concrete software stack used to validate the semantic contracts.

It serves as the **Reference Implementation** for the oracle.

---

## Baseline Identity ([R1])

The [R1] baseline profile is defined as:

* **Distribution:** ROS 2 Jazzy Jalisco
* **Client Library:** `rclcpp` (Standard C++ Client)
* **Ecosystem Stack:** `Nav2` (Navigation 2)

**Commitment:**
Behavior observed in this stack is considered the **de facto standard** for unspecified behavior, unless proven to be a bug.

---

## Version Pinning

Validation runs assume the standard release versions of:
- `ros-jazzy-desktop`
- `ros-jazzy-navigation2`

Exact lockfiles are maintained in the harness directory.
Validation is performed against **containerized binaries**, not source code, to ensure production fidelity.

---

## Scope of Authority

The [R1] baseline is the **final authority** for:
- **Default Behavior:** What happens when specs are silent?
- **Error Codes:** Exact values returned on failure.
- **Timing:** Reasonable bounds for "Settling Window".

The [R1] baseline is **NOT** the authority for:
- **Core Physics:** Invariants defined by `_core.md` files override implementation bugs.
- **Future Distributions:** This baseline applies to Jazzy only.

---

## Divergence Policy

If the Baseline ([R1]) violates a Normative Spec (`SPEC_xx`):

1.  **Verification:** Confirm the spec is derived from upstream/physics (not just an opinion).
2.  **Classification:**
    * **Implementation Bug:** The spec is right; Jazzy is wrong. (Mark scenario as FAIL).
    * **Spec Defect:** The spec is wrong/outdated. (Update spec).
    * **Ambiguity:** The spec is too strict. (Relax spec to [BIC]).

We do **not** silently change specs to match buggy implementations.