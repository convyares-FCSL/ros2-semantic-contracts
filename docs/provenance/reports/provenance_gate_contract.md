# Provenance Gate Contract — Stage-2

## Purpose

This document defines the Stage-2 provenance gate enforcement rules for CI.

---

## Stage Definitions

| Stage | Description | Artifact Requirement |
|-------|-------------|---------------------|
| **Stage 1** | Traceability only | Specs tagged, semantics defined, harness exists |
| **Stage 2** | Execution evidence | EXECUTED scenarios require `result.json` artifacts |
| **Stage 3** | Baseline validation | VALIDATED requires `docker_baseline` artifacts |

**Current Stage: 2**

---

## Rules Enforced (Stage-2)

### R1 — Normative Clauses Must Have Scenario Tags

Any `### SPEC_*` section containing RFC2119 keywords (`MUST`, `MUST NOT`, `SHALL`, `SHALL NOT`, `FORBIDDEN`) must include at least one scenario tag like `[A01]`, `[L07, L08]`.

**Exclusions:** Sections marked with `⚠️ UNVALIDATED` are not counted as normative violations.

### R2 — Referenced Scenario IDs Must Be Defined

All scenario IDs referenced in spec headings must exist in `docs/provenance/scenario/semantics_*.md`.

### R3 — Defined Scenarios Must Have Harness Implementation

All scenario IDs defined in semantics must exist in `harness/scenarios/scenarios_*.json`.

### R4 — EXECUTED Scenarios Must Have Artifacts

For scenarios listed as `EXECUTED` in the execution matrix, the following artifact must exist:

```
evidence/{backend}/{scenario_id}/result.json
```

**Current execution status:**
- `snub`: H01
- `ros_local`: H01, A01, A02
- `docker_baseline`: (not yet enforced)

### R5 — VALIDATED Is Forbidden (Stage-2)

No spec clause may be marked `VALIDATED` unless `docker_baseline` artifacts exist. Since docker baseline is not yet enforced, **VALIDATED is effectively forbidden**.

---

## Artifact Path Convention

```
evidence/
├── snub/
│   └── H01/
│       └── result.json
├── ros_local/
│   ├── H01/
│   │   └── result.json
│   ├── A01/
│   │   └── result.json
│   └── A02/
│       └── result.json
└── docker_baseline/          # Stage 3
    └── {scenario_id}/
        └── result.json
```

### Required `result.json` Fields

```json
{
  "scenario_id": "A01",
  "backend": "ros_local",
  "stage": 2,
  "status": "EXECUTED",
  "timestamp": "2026-02-01T20:08:13Z",
  "notes": "..."
}
```

---

## Execution Matrix

Machine-readable source of truth: `harness/contracts/scenario_execution_matrix.json`

---

## Stage-3 Changes (Future)

When Stage-3 is enabled:

1. `docker_baseline` backend becomes enforced
2. `VALIDATED` markers require artifacts at `evidence/docker_baseline/{scenario_id}/result.json`
3. Comparative multi-language validation may be added

---

## CI Integration

**Workflow:** `.github/workflows/provenance-gate.yml`  
**Script:** `tools/ci/provenance_gate.py`

```yaml
- run: python tools/ci/provenance_gate.py
```

---

**Version:** 2.0.0  
**Last Updated:** 2026-02-01
