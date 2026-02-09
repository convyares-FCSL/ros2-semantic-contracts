"""Parameter operation handlers."""

from typing import Dict, Any, Optional
import rclpy.exceptions
from ..trace_writer import TraceWriter
from ..backend_state import BackendState
from ..errors import BundleError


def exec_declare_param(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute declare_param operation.

    Declares a parameter on the node. Captures ParameterAlreadyDeclaredException
    for redeclaration attempts.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'name' (required): parameter name
            - 'value' (optional): default value (string only for minimal impl)
            - 'descriptor' (optional): NOT IMPLEMENTED for P03

    Returns:
        Result dict:
            Success: {"op": "declare_param", "name": <name>, "declared": true}
            Failure: {"op": "declare_param", "name": <name>, "declared": false, "reason": "AlreadyDeclared"}

    Raises:
        BundleError: Missing 'name' in payload
    """
    # Validate payload
    payload = op.get('payload')
    if not payload or 'name' not in payload:
        raise BundleError(f"declare_param: payload missing 'name' in scenario {scenario_id}")

    name = payload['name']

    # Initialize result
    result = {
        'op': 'declare_param',
        'name': name
    }

    # Extract optional default value (string type only)
    default_value = payload.get('value', '')

    # NOTE: Descriptor parsing (read_only, dynamic_typing) is NOT implemented
    # for minimal P03 support. Future enhancement for P05/P12 scenarios.

    try:
        # CRITICAL: Call real rclpy API - NO SIMULATION
        st.node.declare_parameter(name, default_value)
        result['declared'] = True

    except rclpy.exceptions.ParameterAlreadyDeclaredException:
        # Specific exception for redeclaration - this is what P03 tests
        result['declared'] = False
        result['reason'] = 'AlreadyDeclared'

    except Exception as e:
        # Catch-all for unexpected exceptions (captures semantic divergence)
        result['declared'] = False
        result['reason'] = f'Exception: {str(e)}'

    return result
