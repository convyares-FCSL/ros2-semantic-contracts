"""Parameter operation handlers."""

import time
from typing import Dict, Any, Optional, List
import rclpy
import rclpy.exceptions
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameter_events
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
    ParameterEvent,
)
from ..trace_writer import TraceWriter
from ..backend_state import BackendState
from ..errors import BundleError


def _build_parameter_list(params_list: Any, scenario_id: str, op_name: str) -> List[Parameter]:
    """Build rclpy Parameter list from payload params array."""
    if not isinstance(params_list, list):
        raise BundleError(f"{op_name}: 'params' must be a list in scenario {scenario_id}")

    type_map = {
        'string': Parameter.Type.STRING,
        'integer': Parameter.Type.INTEGER,
        'double': Parameter.Type.DOUBLE,
        'bool': Parameter.Type.BOOL,
    }

    parameters = []
    for idx, param_spec in enumerate(params_list):
        name = param_spec.get('name')
        param_type = param_spec.get('type')
        value = param_spec.get('value')

        if not name:
            raise BundleError(f"{op_name}: param[{idx}] missing 'name' in scenario {scenario_id}")
        if param_type is None:
            raise BundleError(f"{op_name}: param[{idx}] missing 'type' in scenario {scenario_id}")
        if value is None:
            raise BundleError(f"{op_name}: param[{idx}] missing 'value' in scenario {scenario_id}")

        if param_type not in type_map:
            raise BundleError(f"{op_name}: param[{idx}] unsupported type '{param_type}' in scenario {scenario_id}")

        parameters.append(Parameter(name, type_map[param_type], value))

    return parameters


def _parameter_value_to_json(value_msg: Any) -> Dict[str, Any]:
    """Convert ParameterValue message to trace JSON."""
    result: Dict[str, Any] = {'type': int(value_msg.type)}
    if value_msg.type == ParameterType.PARAMETER_BOOL:
        result['value'] = bool(value_msg.bool_value)
    elif value_msg.type == ParameterType.PARAMETER_INTEGER:
        result['value'] = int(value_msg.integer_value)
    elif value_msg.type == ParameterType.PARAMETER_DOUBLE:
        result['value'] = float(value_msg.double_value)
    elif value_msg.type == ParameterType.PARAMETER_STRING:
        result['value'] = value_msg.string_value
    elif value_msg.type == ParameterType.PARAMETER_BYTE_ARRAY:
        result['value'] = list(value_msg.byte_array_value)
    elif value_msg.type == ParameterType.PARAMETER_BOOL_ARRAY:
        result['value'] = list(value_msg.bool_array_value)
    elif value_msg.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        result['value'] = list(value_msg.integer_array_value)
    elif value_msg.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        result['value'] = list(value_msg.double_array_value)
    elif value_msg.type == ParameterType.PARAMETER_STRING_ARRAY:
        result['value'] = list(value_msg.string_array_value)
    return result


def _parameter_to_json(param_msg: Any) -> Dict[str, Any]:
    """Convert Parameter message to trace JSON."""
    value = _parameter_value_to_json(param_msg.value)
    item: Dict[str, Any] = {
        'name': param_msg.name,
        'type': value['type'],
    }
    if 'value' in value:
        item['value'] = value['value']
    return item


def exec_declare_param(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute declare_param operation.

    Declares a parameter on the node with optional descriptor.
    Captures ParameterAlreadyDeclaredException for redeclaration attempts.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'name' (required): parameter name
            - 'value' (optional): default value (string type)
            - 'descriptor' (optional): dict with 'read_only', 'dynamic_typing' fields

    Returns:
        Result dict:
            Success: {"op": "declare_param", "name": <name>, "declared": true, "descriptor": {...}}
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

    # Parse descriptor (optional)
    descriptor = ParameterDescriptor()
    if 'descriptor' in payload:
        desc_dict = payload['descriptor']
        if 'read_only' in desc_dict:
            descriptor.read_only = desc_dict['read_only']
        if 'dynamic_typing' in desc_dict:
            descriptor.dynamic_typing = desc_dict['dynamic_typing']

    try:
        # CRITICAL: Call real rclpy API with descriptor - NO SIMULATION
        st.node.declare_parameter(name, default_value, descriptor)
        result['declared'] = True

        # Echo descriptor for trace inspection (matching backend_prod)
        if descriptor.read_only:
            result['descriptor'] = {'read_only': True}

    except rclpy.exceptions.ParameterAlreadyDeclaredException:
        # Specific exception for redeclaration - this is what P03 tests
        result['declared'] = False
        result['reason'] = 'AlreadyDeclared'

    except Exception as e:
        # Catch-all for unexpected exceptions (captures semantic divergence)
        result['declared'] = False
        result['reason'] = f'Exception: {str(e)}'

    return result


def exec_set_param(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute set_param operation.

    Sets a single parameter value. Captures ParameterNotDeclaredException
    when attempting to set undeclared parameters.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'name' (required): parameter name
            - 'type' (required): parameter type (string/integer/double/bool)
            - 'value' (required): value to set

    Returns:
        Result dict:
            Success: {"op": "set_param", "name": <name>, "successful": true}
            Failure: {"op": "set_param", "name": <name>, "successful": false, "reason": <reason>}

    Raises:
        BundleError: Missing required payload fields
    """
    # Validate payload
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"set_param: missing payload in scenario {scenario_id}")

    name = payload.get('name')
    param_type = payload.get('type')
    value = payload.get('value')

    if not name:
        raise BundleError(f"set_param: payload missing 'name' in scenario {scenario_id}")
    if param_type is None:
        raise BundleError(f"set_param: payload missing 'type' in scenario {scenario_id}")
    if value is None:
        raise BundleError(f"set_param: payload missing 'value' in scenario {scenario_id}")

    # Initialize result
    result = {
        'op': 'set_param',
        'name': name
    }

    # Map string type to rclpy Parameter.Type
    type_map = {
        'string': Parameter.Type.STRING,
        'integer': Parameter.Type.INTEGER,
        'double': Parameter.Type.DOUBLE,
        'bool': Parameter.Type.BOOL,
    }

    if param_type not in type_map:
        raise BundleError(f"set_param: unsupported type '{param_type}' in scenario {scenario_id}")

    try:
        # CRITICAL: Call real rclpy API - NO SIMULATION
        param = Parameter(name, type_map[param_type], value)
        set_result = st.node.set_parameters([param])

        # Check result (list of SetParametersResult)
        if set_result and len(set_result) > 0:
            if set_result[0].successful:
                result['successful'] = True
            else:
                result['successful'] = False
                result['reason'] = set_result[0].reason or "SetFailed"
        else:
            # Unexpected: no result returned
            result['successful'] = False
            result['reason'] = "NoResult"

    except rclpy.exceptions.ParameterNotDeclaredException:
        # Expected failure for P06 - parameter was never declared
        result['successful'] = False
        result['reason'] = 'NotDeclared'

    except Exception as e:
        # Catch-all for unexpected exceptions (captures semantic divergence)
        result['successful'] = False
        result['reason'] = f'Exception: {str(e)}'

    return result


def exec_get_param(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute get_param operation.

    Retrieves a parameter value and metadata. Returns declaration status,
    type, and value if set.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'name' (required): parameter name

    Returns:
        Result dict with fields:
            - op: "get_param"
            - name: parameter name
            - declared: bool (whether parameter is declared)
            - set: bool (whether parameter has a value)
            - type: string (parameter type or "NOT_SET")
            - value: actual value (only if set=true)

    Raises:
        BundleError: Missing required payload fields
    """
    # Validate payload
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"get_param: missing payload in scenario {scenario_id}")

    name = payload.get('name')
    if not name:
        raise BundleError(f"get_param: payload missing 'name' in scenario {scenario_id}")

    # Initialize result
    result = {
        'op': 'get_param',
        'name': name
    }

    # Check if parameter is declared
    if not st.node.has_parameter(name):
        result['declared'] = False
        result['set'] = False
        result['type'] = 'NOT_SET'
        return result

    # Parameter is declared - get it
    result['declared'] = True
    param = st.node.get_parameter(name)

    # Check if parameter has a value
    if param.type_ == Parameter.Type.NOT_SET:
        result['set'] = False
        result['type'] = 'NOT_SET'
        return result

    # Parameter is set - extract type and value
    result['set'] = True

    # Map rclpy Parameter.Type to string
    type_map = {
        Parameter.Type.BOOL: 'bool',
        Parameter.Type.INTEGER: 'integer',
        Parameter.Type.DOUBLE: 'double',
        Parameter.Type.STRING: 'string',
        Parameter.Type.BYTE_ARRAY: 'byte_array',
        Parameter.Type.BOOL_ARRAY: 'bool_array',
        Parameter.Type.INTEGER_ARRAY: 'integer_array',
        Parameter.Type.DOUBLE_ARRAY: 'double_array',
        Parameter.Type.STRING_ARRAY: 'string_array',
    }

    result['type'] = type_map.get(param.type_, 'unknown')
    result['value'] = param.value

    return result


def exec_set_params_batch(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute set_params_batch operation.

    Sets multiple parameters atomically using set_parameters_atomically().
    All parameters must succeed or the entire batch fails with no changes.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'params' (required): list of parameter dicts, each with:
                - 'name' (required): parameter name
                - 'type' (required): parameter type
                - 'value' (required): value to set

    Returns:
        Result dict:
            Success: {"op": "set_params_batch", "count": N, "successful": true}
            Failure: {"op": "set_params_batch", "count": N, "successful": false, "reason": <reason>}

    Raises:
        BundleError: Missing required payload fields or invalid structure
    """
    # Validate payload
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"set_params_batch: missing payload in scenario {scenario_id}")

    params_list = payload.get('params')
    if params_list is None:
        raise BundleError(f"set_params_batch: payload missing 'params' in scenario {scenario_id}")

    parameters = _build_parameter_list(params_list, scenario_id, 'set_params_batch')

    # Initialize result
    result = {
        'op': 'set_params_batch',
        'count': len(parameters)
    }

    try:
        # CRITICAL: Call real rclpy API - set_parameters_atomically
        # This MUST be atomic: all succeed or all fail with no partial application
        set_result = st.node.set_parameters_atomically(parameters)

        # set_parameters_atomically returns a single SetParametersResult
        result['successful'] = set_result.successful
        if not set_result.successful and set_result.reason:
            result['reason'] = set_result.reason

    except rclpy.exceptions.ParameterNotDeclaredException:
        # Expected failure - one or more parameters not declared
        result['successful'] = False
        result['reason'] = 'NotDeclared'

    except Exception as e:
        # Catch-all for unexpected exceptions
        result['successful'] = False
        result['reason'] = f'Exception: {str(e)}'

    return result


def exec_set_params_non_atomic(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute set_params_non_atomic using Node.set_parameters()."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"set_params_non_atomic: missing payload in scenario {scenario_id}")

    params_list = payload.get('params')
    if params_list is None:
        raise BundleError(f"set_params_non_atomic: payload missing 'params' in scenario {scenario_id}")

    parameters = _build_parameter_list(params_list, scenario_id, 'set_params_non_atomic')

    result: Dict[str, Any] = {
        'op': 'set_params_non_atomic',
        'count': len(parameters),
        'successful': True,
        'results': [],
    }

    try:
        set_results = st.node.set_parameters(parameters)
        all_successful = True
        per_param: List[Dict[str, Any]] = []
        for idx, set_result in enumerate(set_results):
            item: Dict[str, Any] = {
                'name': parameters[idx].name,
                'successful': bool(set_result.successful),
            }
            if not set_result.successful and set_result.reason:
                item['reason'] = set_result.reason
            if not set_result.successful:
                all_successful = False
            per_param.append(item)
        result['successful'] = all_successful
        result['results'] = per_param
    except rclpy.exceptions.ParameterNotDeclaredException:
        result['successful'] = False
        result['reason'] = 'NotDeclared'
    except Exception as e:
        result['successful'] = False
        result['reason'] = f'Exception: {str(e)}'

    return result


def exec_register_param_callback(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Register add_on_set_parameters_callback with reject-name mode."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"register_param_callback: missing payload in scenario {scenario_id}")

    mode = payload.get('mode')
    if mode != 'reject_names':
        raise BundleError(f"register_param_callback: unsupported mode '{mode}' in scenario {scenario_id}")

    reject = payload.get('reject')
    if not isinstance(reject, list) or not all(isinstance(name, str) for name in reject):
        raise BundleError(f"register_param_callback: payload.reject must be a string list in scenario {scenario_id}")

    reject_names = set(reject)

    if st.on_set_param_callback is not None:
        st.node.remove_on_set_parameters_callback(st.on_set_param_callback)
        st.on_set_param_callback = None

    def _callback(params: List[Parameter]) -> SetParametersResult:
        out = SetParametersResult()
        out.successful = True
        for param in params:
            if param.name in reject_names:
                out.successful = False
                out.reason = f"RejectedByCallback:{param.name}"
                return out
        return out

    st.node.add_on_set_parameters_callback(_callback)
    st.on_set_param_callback = _callback

    return {
        'op': 'register_param_callback',
        'mode': mode,
        'reject': reject,
        'registered': True,
    }


def exec_register_pre_set_callback(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Register add_pre_set_parameters_callback with remove-name mode."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"register_pre_set_callback: missing payload in scenario {scenario_id}")

    mode = payload.get('mode')
    if mode != 'remove_names':
        raise BundleError(f"register_pre_set_callback: unsupported mode '{mode}' in scenario {scenario_id}")

    remove = payload.get('remove')
    if not isinstance(remove, list) or not all(isinstance(name, str) for name in remove):
        raise BundleError(f"register_pre_set_callback: payload.remove must be a string list in scenario {scenario_id}")

    remove_names = set(remove)

    if st.pre_set_param_callback is not None:
        st.node.remove_pre_set_parameters_callback(st.pre_set_param_callback)
        st.pre_set_param_callback = None

    def _callback(params: List[Parameter]) -> List[Parameter]:
        return [param for param in params if param.name not in remove_names]

    st.node.add_pre_set_parameters_callback(_callback)
    st.pre_set_param_callback = _callback

    return {
        'op': 'register_pre_set_callback',
        'mode': mode,
        'remove': remove,
        'registered': True,
    }


def exec_observe_param_events(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Observe /parameter_events and emit parameter_event trace entries."""
    wait_ms = op.get('ms')
    if wait_ms is None:
        wait_ms = (op.get('payload') or {}).get('ms', 100)
    if not isinstance(wait_ms, int) or wait_ms < 0:
        raise BundleError(f"observe_param_events: ms must be a non-negative integer in scenario {scenario_id}")

    if st.param_event_sub is None:
        def _param_event_cb(msg: ParameterEvent) -> None:
            st.param_event_buffer.append({
                'type': 'parameter_event',
                'scenario_id': scenario_id,
                'node': msg.node,
                'stamp': {'sec': msg.stamp.sec, 'nanosec': msg.stamp.nanosec},
                'new_parameters': [_parameter_to_json(p) for p in msg.new_parameters],
                'changed_parameters': [_parameter_to_json(p) for p in msg.changed_parameters],
                'deleted_parameters': [_parameter_to_json(p) for p in msg.deleted_parameters],
            })

        st.param_event_sub = st.node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            _param_event_cb,
            qos_profile_parameter_events,
        )

    deadline = time.monotonic() + (wait_ms / 1000.0)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(st.node, timeout_sec=min(0.05, max(remaining, 0.0)))

    drained = list(st.param_event_buffer)
    st.param_event_buffer.clear()

    for ev in drained:
        w.emit(ev)

    return {
        'op': 'observe_param_events',
        'window_ms': wait_ms,
        'count': len(drained),
    }


def exec_list_params(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """List declared parameters with optional prefix/depth filtering."""
    payload = op.get('payload') or {}

    prefixes = payload.get('prefixes', [])
    if not isinstance(prefixes, list) or not all(isinstance(p, str) for p in prefixes):
        raise BundleError(f"list_params: payload.prefixes must be a string list in scenario {scenario_id}")

    depth = payload.get('depth', 0)
    if not isinstance(depth, int) or depth < 0:
        raise BundleError(f"list_params: payload.depth must be a non-negative integer in scenario {scenario_id}")

    listed = st.node.list_parameters(prefixes, depth)
    return {
        'op': 'list_params',
        'prefixes_in': prefixes,
        'depth': depth,
        'names': list(listed.names),
        'prefixes': list(listed.prefixes),
        'count': len(listed.names),
    }


def exec_describe_param(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute describe_param operation.

    Describes a parameter and returns selected descriptor fields.
    For undeclared parameters, returns a default descriptor with
    successful=false to match backend_prod behavior.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'name' (required): parameter name

    Returns:
        Result dict with fields:
            - op: "describe_param"
            - name: parameter name
            - successful: bool
            - reason: error string when unsuccessful
            - descriptor: {type, read_only, dynamic_typing}

    Raises:
        BundleError: Missing required payload fields
    """
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"describe_param: missing payload in scenario {scenario_id}")

    name = payload.get('name')
    if not name:
        raise BundleError(f"describe_param: payload missing 'name' in scenario {scenario_id}")

    result = {
        'op': 'describe_param',
        'name': name
    }

    descriptor = ParameterDescriptor()

    try:
        descriptors = st.node.describe_parameters([name])
        if not descriptors:
            raise BundleError('describe_parameters returned empty vector')
        descriptor = descriptors[0]
        result['successful'] = True
    except rclpy.exceptions.ParameterNotDeclaredException:
        result['successful'] = False
        # backend_prod surfaces the undeclared parameter name in reason.
        result['reason'] = name
    except Exception as e:
        result['successful'] = False
        result['reason'] = str(e)
        # Keep default-constructed descriptor (type=0/NOT_SET)

    result['descriptor'] = {
        'type': int(descriptor.type),
        'read_only': bool(descriptor.read_only),
        'dynamic_typing': bool(descriptor.dynamic_typing),
    }

    return result


def exec_undeclare_param(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute undeclare_param operation.

    Removes a previously declared parameter from the node.

    Args:
        w: TraceWriter instance (unused, signature consistency)
        st: BackendState with initialized node
        scenario_id: Scenario identifier (for error messages)
        op: Operation dict with 'payload' containing:
            - 'name' (required): parameter name

    Returns:
        Result dict:
            Success: {"op": "undeclare_param", "name": <name>, "successful": true}
            Failure: {"op": "undeclare_param", "name": <name>, "successful": false, "reason": <reason>}

    Raises:
        BundleError: Missing required payload fields
    """
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"undeclare_param: missing payload in scenario {scenario_id}")

    name = payload.get('name')
    if not name:
        raise BundleError(f"undeclare_param: payload missing 'name' in scenario {scenario_id}")

    result = {
        'op': 'undeclare_param',
        'name': name
    }

    try:
        st.node.undeclare_parameter(name)
        result['successful'] = True
    except rclpy.exceptions.ParameterNotDeclaredException:
        result['successful'] = False
        result['reason'] = 'NotDeclared'
    except Exception as e:
        result['successful'] = False
        result['reason'] = f'Exception: {str(e)}'

    return result
