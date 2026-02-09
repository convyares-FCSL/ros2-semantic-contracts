"""JSON bundle parsing and validation."""

import json
from typing import Dict, Any
from .errors import BundleError, SystemError


def parse_bundle(path: str) -> Dict[str, Any]:
    """Parse and validate scenario bundle JSON.
    
    Args:
        path: Path to bundle JSON file
        
    Returns:
        Parsed bundle dict
        
    Raises:
        BundleError: Invalid JSON or missing scenarios
        SystemError: File I/O failure
    """
    try:
        with open(path, 'r') as f:
            bundle = json.load(f)
    except FileNotFoundError:
        raise SystemError(f"Bundle file not found: {path}")
    except IOError as e:
        raise SystemError(f"Failed to read bundle: {e}")
    except json.JSONDecodeError as e:
        raise BundleError(f"Invalid JSON in bundle: {e}")
    
    # Validate root structure
    if not isinstance(bundle, dict):
        raise BundleError("Bundle root must be a JSON object")
    
    if 'scenarios' not in bundle:
        raise BundleError("Bundle missing 'scenarios' field")
    
    if not isinstance(bundle['scenarios'], dict):
        raise BundleError("Bundle 'scenarios' must be an object")
    
    return bundle
