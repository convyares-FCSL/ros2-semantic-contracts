#!/usr/bin/env python3
"""
Provenance Gate — Stage-2 CI enforcement of traceability rules.

Stage-2 Rules:
  R1: Normative clauses (MUST/SHALL) must have scenario tags [Xnn]
  R2: Referenced scenario IDs must be defined in semantics files
  R3: Defined scenarios must have harness implementation in JSON
  R4: EXECUTED scenarios (per matrix) should have evidence artifacts (WARN ONLY in stage-2)
  R5: VALIDATED is forbidden unless docker_baseline artifacts exist

Evidence Policy (Stage-2):
  During active harness development, execution artifacts live in /tmp and CI;
  they are NOT committed to the repo. Evidence receipts become required
  when backend_prod/docker_baseline is introduced (Stage-3+).

Current execution status (Stage-2):
  snub: H01
  ros_local: H01, A01, A02
  docker_baseline: not yet enforced
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

SPEC_DIR = ROOT / "docs" / "spec"
SEMANTICS_DIR = ROOT / "docs" / "provenance" / "scenario"
HARNESS_SCENARIOS_DIR = ROOT / "harness" / "scenarios"
EXECUTION_MATRIX = ROOT / "harness" / "contracts" / "scenario_execution_matrix.json"
EVIDENCE_DIR = ROOT / "evidence"

RFC2119 = re.compile(r"\b(MUST|MUST NOT|FORBIDDEN|SHALL|SHALL NOT)\b")
SPEC_HEADING = re.compile(r"^###\s+(SPEC_[A-Z0-9_]+)\b(.*)$")
SCENARIO_TAGS = re.compile(r"\[([A-Z]\d{2}(?:\s*,\s*[A-Z]\d{2})*)\]")
SCENARIO_ID = re.compile(r"\b([AGLPSH]\d{2})\b")

# Match "VALIDATED" but NOT "UNVALIDATED"
VALIDATED_PATTERN = re.compile(r"(?<!UN)VALIDATED")
# Match UNVALIDATED blocks to exclude
UNVALIDATED_BLOCK = re.compile(r"⚠️\s*\*?\*?UNVALIDATED")


def read_text(p: Path) -> str:
    return p.read_text(encoding="utf-8", errors="replace")


def iter_md_files(base: Path):
    if not base.exists():
        return
    for p in base.rglob("*.md"):
        yield p


def load_execution_matrix() -> dict:
    """Load the stage-2 execution matrix."""
    if not EXECUTION_MATRIX.exists():
        return {"backends": {}}
    return json.loads(EXECUTION_MATRIX.read_text())


def collect_defined_scenarios() -> set[str]:
    """Collect scenario IDs from semantics files (### Xnn headings)."""
    defined = set()
    for p in iter_md_files(SEMANTICS_DIR):
        if "semantics_" in p.name:
            text = read_text(p)
            for m in re.finditer(r"^### ([A-Z]\d{2})\b", text, re.MULTILINE):
                defined.add(m.group(1))
    return defined


def collect_harness_scenarios() -> set[str]:
    """Collect scenario IDs from harness JSON files."""
    implemented = set()
    for f in HARNESS_SCENARIOS_DIR.glob("*.json"):
        try:
            data = json.loads(f.read_text())
            if "scenarios" in data:
                for key in data["scenarios"].keys():
                    m = re.match(r"^([A-Z]\d{2})_", key)
                    if m:
                        implemented.add(m.group(1))
        except (json.JSONDecodeError, KeyError):
            pass
    return implemented


def is_in_unvalidated_context(body_text: str) -> bool:
    """Check if body text is inside an UNVALIDATED block."""
    return bool(UNVALIDATED_BLOCK.search(body_text))


def check_specs_have_scenario_tags() -> tuple[list[str], int, int]:
    """R1: If a SPEC_* section contains RFC2119 keywords, heading must have [Xnn].
    
    Returns: (errors, total_normative_clauses, clauses_missing_tags)
    """
    errors = []
    total_normative = 0
    missing_tags = 0
    
    for p in iter_md_files(SPEC_DIR):
        lines = read_text(p).splitlines()
        i = 0
        while i < len(lines):
            m = SPEC_HEADING.match(lines[i])
            if not m:
                i += 1
                continue

            spec_id = m.group(1)
            heading_line = lines[i]
            j = i + 1
            body = []
            while j < len(lines) and not lines[j].startswith("### "):
                body.append(lines[j])
                j += 1
            body_text = "\n".join(body)

            # Skip UNVALIDATED sections for normative counting
            if is_in_unvalidated_context(body_text):
                i = j
                continue

            if RFC2119.search(body_text):
                total_normative += 1
                if not SCENARIO_TAGS.search(heading_line):
                    missing_tags += 1
                    errors.append(
                        f"{p.relative_to(ROOT)}:{i+1}: {spec_id} contains RFC2119 keywords but has no [ScenarioIDs] in heading"
                    )

            i = j
    return errors, total_normative, missing_tags


def check_referenced_scenarios_defined(defined: set[str]) -> tuple[list[str], set[str]]:
    """R2: Scenario IDs referenced in specs must be defined in semantics files.
    
    Returns: (errors, all_referenced_ids)
    """
    errors = []
    all_referenced = set()
    
    for p in iter_md_files(SPEC_DIR):
        lines = read_text(p).splitlines()
        for i, line in enumerate(lines, 1):
            m = SPEC_HEADING.match(line)
            if m:
                tags_match = SCENARIO_TAGS.search(line)
                if tags_match:
                    tags = [t.strip() for t in re.findall(r"[A-Z]\d{2}", tags_match.group(1))]
                    for tag in tags:
                        all_referenced.add(tag)
                        if tag not in defined:
                            errors.append(
                                f"{p.relative_to(ROOT)}:{i}: Scenario {tag} referenced but not defined in semantics files"
                            )
    return errors, all_referenced


def check_defined_scenarios_have_harness_impl(defined: set[str], harness: set[str]) -> list[str]:
    """R3: Defined scenarios must have harness implementation."""
    errors = []
    missing = defined - harness
    for sid in sorted(missing):
        errors.append(f"Scenario {sid} defined in semantics but missing from harness JSON")
    return errors


def check_executed_have_artifacts(matrix: dict) -> tuple[list[str], dict]:
    """R4: EXECUTED scenarios should have evidence artifacts.
    
    NOTE: In stage-2, this is a WARNING only (ephemeral /tmp evidence).
    In stage-3+, missing artifacts will cause gate failure.
    
    Returns: (warnings, execution_status_dict)
    """
    errors = []
    status = {}
    
    backends = matrix.get("backends", {})
    for backend_name, backend_info in backends.items():
        executed = backend_info.get("executed", [])
        status[backend_name] = {"executed": executed, "artifacts_ok": [], "artifacts_missing": []}
        
        for sid in executed:
            artifact_path = EVIDENCE_DIR / backend_name / sid / "result.json"
            if artifact_path.exists():
                status[backend_name]["artifacts_ok"].append(sid)
            else:
                status[backend_name]["artifacts_missing"].append(sid)
                errors.append(
                    f"EXECUTED scenario {sid} on {backend_name} missing artifact: evidence/{backend_name}/{sid}/result.json"
                )
    
    return errors, status


def check_validated_forbidden() -> list[str]:
    """R5: VALIDATED is forbidden unless docker_baseline artifacts exist."""
    errors = []
    
    for p in iter_md_files(SPEC_DIR):
        lines = read_text(p).splitlines()
        i = 0
        while i < len(lines):
            m = SPEC_HEADING.match(lines[i])
            if not m:
                i += 1
                continue

            heading_line = lines[i]
            j = i + 1
            body = []
            while j < len(lines) and not lines[j].startswith("### "):
                body.append(lines[j])
                j += 1
            body_text = "\n".join(body)

            # Check for true VALIDATED (not UNVALIDATED)
            if VALIDATED_PATTERN.search(body_text):
                tags = SCENARIO_TAGS.search(heading_line)
                if tags:
                    ids = [x.strip() for x in re.findall(r"[A-Z]\d{2}", tags.group(1))]
                    for sid in ids:
                        # Stage-2: VALIDATED requires docker_baseline evidence
                        docker_artifact = EVIDENCE_DIR / "docker_baseline" / sid / "result.json"
                        if not docker_artifact.exists():
                            errors.append(
                                f"{p.relative_to(ROOT)}:{i+1}: VALIDATED {sid} forbidden — docker_baseline artifact required"
                            )

            i = j
    return errors


def main() -> int:
    errors = []
    
    # Load matrix
    matrix = load_execution_matrix()
    
    # Collect definitions
    defined = collect_defined_scenarios()
    harness = collect_harness_scenarios()

    # Run checks
    tag_errors, total_normative, missing_tags = check_specs_have_scenario_tags()
    errors += tag_errors
    
    ref_errors, all_referenced = check_referenced_scenarios_defined(defined)
    errors += ref_errors
    
    errors += check_defined_scenarios_have_harness_impl(defined, harness)
    
    # R4 is non-blocking in stage-2 (ephemeral evidence policy)
    artifact_warnings, exec_status = check_executed_have_artifacts(matrix)
    
    errors += check_validated_forbidden()

    # Report
    print("=" * 60)
    print("PROVENANCE GATE — STAGE 2")
    print("=" * 60)
    print(f"\nNormative clauses scanned: {total_normative}")
    print(f"Clauses missing tags: {missing_tags}")
    print(f"Scenario IDs referenced: {len(all_referenced)}")
    print(f"Scenarios defined in semantics: {len(defined)}")
    print(f"Scenarios in harness: {len(harness)}")
    print(f"Missing semantics definitions: {len(all_referenced - defined)}")
    print(f"Missing harness implementations: {len(defined - harness)}")
    
    print("\nExecution status per backend (artifacts):")
    for backend, status in exec_status.items():
        ok = len(status["artifacts_ok"])
        missing = len(status["artifacts_missing"])
        label = "WARN" if missing > 0 else "OK"
        print(f"  {backend}: {ok} OK, {missing} missing [{label}]")
    
    if artifact_warnings:
        print("\n[STAGE-2 INFO] Artifact warnings (non-blocking):")
        for w in artifact_warnings:
            print(f"  ⚠ {w}")
    
    if errors:
        print("\n" + "=" * 60)
        print("PROVENANCE GATE FAILED")
        print("=" * 60 + "\n")
        for e in errors:
            print(f"  ✗ {e}")
        return 1

    print("\n" + "=" * 60)
    print("PROVENANCE GATE PASSED")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
