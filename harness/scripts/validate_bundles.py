#!/usr/bin/env python3
"""Validate scenario bundle JSON files have required fields.

This script fails with clear errors if any scenario is missing required fields.
Used in CI to prevent schema regressions.
"""

import json
import re
import sys
from pathlib import Path

HARNESS_DIR = Path(__file__).parent.parent
SCENARIOS_DIR = HARNESS_DIR / "scenarios"

REQUIRED_FIELDS = ["spec_id", "title", "layer", "ops", "expects"]
OPTIONAL_FIELDS = ["notes", "requires"]

def validate_bundle(path: Path) -> list[str]:
    """Validate a scenario bundle file. Returns list of errors."""
    errors = []
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError as e:
        errors.append(f"{path.name}: Invalid JSON: {e}")
        return errors

    scenarios = data.get("scenarios", {})
    for scenario_key, scenario in scenarios.items():
        for field in REQUIRED_FIELDS:
            if field not in scenario:
                errors.append(
                    f"{path.name}: {scenario_key} missing required field '{field}'"
                )
        
        # Check for legacy fields that should not exist
        if "spec_ref" in scenario:
            errors.append(
                f"{path.name}: {scenario_key} uses legacy 'spec_ref' (use 'spec_id')"
            )
        if "description" in scenario and "title" not in scenario:
            errors.append(
                f"{path.name}: {scenario_key} uses legacy 'description' (use 'title')"
            )
    
    return errors


# Action scenario ID pattern (matches A01_, A02_, etc.)
_ACTION_ID = re.compile(r"^A\d{2}_")

# Copy-paste JSON shape shown in BOUNDARY-A-01 errors for absent boundary.
_AXX_BOUNDARY_SHAPE = (
    '      "boundary": {\n'
    '          "sut": "client",\n'
    '          "peer": "external",\n'
    '          "peer_rule": "Action server must be provided by the scenario '
    'environment. Backend must not host or configure server accept/reject policy."\n'
    '      }'
)


def check_boundaries(path: Path, data: dict) -> list[str]:
    """BOUNDARY-A-01: Axx scenarios must declare boundary metadata.

    Fires only on Action scenarios (ID matches A\\d{2}_).  Violations
    are errors; the pipeline does not pass.
    """
    errors = []
    for scenario_key, scenario in data.get("scenarios", {}).items():
        if not _ACTION_ID.match(scenario_key):
            continue
        boundary = scenario.get("boundary")
        if boundary is None:
            errors.append(
                f"{path.name}: {scenario_key}: BOUNDARY-A-01 [ERROR] "
                f"no boundary metadata declared. Add to scenario:\n"
                f"{_AXX_BOUNDARY_SHAPE}\n"
                f"    See docs/provenance/oracle/boundaries.md"
            )
        elif boundary.get("peer") != "external":
            errors.append(
                f"{path.name}: {scenario_key}: BOUNDARY-A-01 [ERROR] "
                f"boundary.peer is \"{boundary.get('peer')}\" but must be "
                f"\"external\" for Action scenarios. "
                f"See docs/provenance/oracle/boundaries.md §3"
            )
    return errors


def main() -> int:
    if not SCENARIOS_DIR.exists():
        print(f"ERROR: Scenarios directory not found: {SCENARIOS_DIR}", file=sys.stderr)
        return 1

    all_errors = []
    bundle_count = 0
    scenario_count = 0

    for bundle_path in sorted(SCENARIOS_DIR.glob("scenarios_*.json")):
        bundle_count += 1
        data = json.loads(bundle_path.read_text())
        scenario_count += len(data.get("scenarios", {}))
        errors = validate_bundle(bundle_path)
        all_errors.extend(errors)
        all_errors.extend(check_boundaries(bundle_path, data))

    print(f"Validated {bundle_count} bundles, {scenario_count} scenarios")

    if all_errors:
        print(f"\n{len(all_errors)} ERROR(S) FOUND:\n", file=sys.stderr)
        for err in all_errors:
            print(f"  ✗ {err}", file=sys.stderr)
        return 1

    print("✓ All scenario bundles valid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
