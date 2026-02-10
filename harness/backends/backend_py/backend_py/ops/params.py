"""Parameter operation handlers."""

import threading
import time
from typing import Dict, Any, Optional, List
import rclpy
import rclpy.exceptions
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameter_events
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
    IntegerRange,
    FloatingPointRange,
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
        'byte_array': Parameter.Type.BYTE_ARRAY,
        'bool_array': Parameter.Type.BOOL_ARRAY,
        'integer_array': Parameter.Type.INTEGER_ARRAY,
        'double_array': Parameter.Type.DOUBLE_ARRAY,
        'string_array': Parameter.Type.STRING_ARRAY,
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


def _parameter_to_string(param: Parameter) -> str:
    """Stringify parameter value for concurrent snapshot comparisons."""
    if param.type_ == Parameter.Type.BOOL:
        return 'true' if param.value else 'false'
    if param.type_ == Parameter.Type.INTEGER:
        return str(int(param.value))
    if param.type_ == Parameter.Type.DOUBLE:
        return str(float(param.value))
    if param.type_ == Parameter.Type.STRING:
        return str(param.value)
    if param.type_ == Parameter.Type.NOT_SET:
        return 'NOT_SET'
    return 'UNSUPPORTED'


def _descriptor_to_json(descriptor: ParameterDescriptor) -> Dict[str, Any]:
    """Convert ParameterDescriptor message to trace JSON."""
    return {
        'type': int(descriptor.type),
        'read_only': bool(descriptor.read_only),
        'dynamic_typing': bool(descriptor.dynamic_typing),
        'description': descriptor.description,
        'additional_constraints': descriptor.additional_constraints,
        'integer_range': [
            {
                'from_value': int(r.from_value),
                'to_value': int(r.to_value),
                'step': int(r.step),
            }
            for r in descriptor.integer_range
        ],
        'floating_point_range': [
            {
                'from_value': float(r.from_value),
                'to_value': float(r.to_value),
                'step': float(r.step),
            }
            for r in descriptor.floating_point_range
        ],
    }


def _cli_assignment_from_json_value(name: str, value: Any, scenario_id: str) -> str:
    if isinstance(value, bool):
        rendered = 'true' if value else 'false'
    elif isinstance(value, int):
        rendered = str(value)
    elif isinstance(value, float):
        rendered = str(value)
    elif isinstance(value, str):
        rendered = value
    else:
        raise BundleError(
            "probe_cli_typed_no_default_declare: payload.cli_value must be bool/int/float/string "
            f"in scenario {scenario_id}"
        )
    return f'{name}:={rendered}'


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

    # Parse descriptor (optional)
    descriptor = ParameterDescriptor()
    explicit_descriptor_type: Optional[int] = None
    if 'descriptor' in payload:
        desc_dict = payload['descriptor']
        if not isinstance(desc_dict, dict):
            raise BundleError(f"declare_param: payload.descriptor must be object in scenario {scenario_id}")
        if 'read_only' in desc_dict:
            if not isinstance(desc_dict['read_only'], bool):
                raise BundleError(
                    f"declare_param: payload.descriptor.read_only must be bool in scenario {scenario_id}"
                )
            descriptor.read_only = desc_dict['read_only']
        if 'dynamic_typing' in desc_dict:
            if not isinstance(desc_dict['dynamic_typing'], bool):
                raise BundleError(
                    f"declare_param: payload.descriptor.dynamic_typing must be bool in scenario {scenario_id}"
                )
            descriptor.dynamic_typing = desc_dict['dynamic_typing']
        if 'type' in desc_dict:
            if not isinstance(desc_dict['type'], int):
                raise BundleError(
                    f"declare_param: payload.descriptor.type must be int in scenario {scenario_id}"
                )
            explicit_descriptor_type = int(desc_dict['type'])
            descriptor.type = explicit_descriptor_type
        if 'description' in desc_dict:
            if not isinstance(desc_dict['description'], str):
                raise BundleError(
                    f"declare_param: payload.descriptor.description must be string in scenario {scenario_id}"
                )
            descriptor.description = desc_dict['description']
        if 'additional_constraints' in desc_dict:
            if not isinstance(desc_dict['additional_constraints'], str):
                raise BundleError(
                    "declare_param: payload.descriptor.additional_constraints must be string "
                    f"in scenario {scenario_id}"
                )
            descriptor.additional_constraints = desc_dict['additional_constraints']
        if 'integer_range' in desc_dict:
            integer_ranges = desc_dict['integer_range']
            if not isinstance(integer_ranges, list):
                raise BundleError(
                    f"declare_param: payload.descriptor.integer_range must be list in scenario {scenario_id}"
                )
            descriptor.integer_range = []
            for idx, item in enumerate(integer_ranges):
                if not isinstance(item, dict):
                    raise BundleError(
                        "declare_param: payload.descriptor.integer_range[] must be object "
                        f"in scenario {scenario_id}"
                    )
                if 'from_value' not in item or 'to_value' not in item or 'step' not in item:
                    raise BundleError(
                        "declare_param: payload.descriptor.integer_range[] requires from_value,to_value,step "
                        f"in scenario {scenario_id}"
                    )
                if not isinstance(item['from_value'], int) or not isinstance(item['to_value'], int):
                    raise BundleError(
                        "declare_param: payload.descriptor.integer_range[].from_value/to_value must be int "
                        f"in scenario {scenario_id}"
                    )
                if not isinstance(item['step'], int) or item['step'] < 0:
                    raise BundleError(
                        "declare_param: payload.descriptor.integer_range[].step must be non-negative int "
                        f"in scenario {scenario_id}"
                    )
                descriptor.integer_range.append(
                    IntegerRange(
                        from_value=int(item['from_value']),
                        to_value=int(item['to_value']),
                        step=int(item['step']),
                    )
                )
        if 'floating_point_range' in desc_dict:
            float_ranges = desc_dict['floating_point_range']
            if not isinstance(float_ranges, list):
                raise BundleError(
                    "declare_param: payload.descriptor.floating_point_range must be list "
                    f"in scenario {scenario_id}"
                )
            descriptor.floating_point_range = []
            for item in float_ranges:
                if not isinstance(item, dict):
                    raise BundleError(
                        "declare_param: payload.descriptor.floating_point_range[] must be object "
                        f"in scenario {scenario_id}"
                    )
                if 'from_value' not in item or 'to_value' not in item or 'step' not in item:
                    raise BundleError(
                        "declare_param: payload.descriptor.floating_point_range[] requires from_value,to_value,step "
                        f"in scenario {scenario_id}"
                    )
                if (
                    not isinstance(item['from_value'], (int, float))
                    or not isinstance(item['to_value'], (int, float))
                    or not isinstance(item['step'], (int, float))
                ):
                    raise BundleError(
                        "declare_param: payload.descriptor.floating_point_range[].from_value/to_value/step "
                        f"must be numeric in scenario {scenario_id}"
                    )
                descriptor.floating_point_range.append(
                    FloatingPointRange(
                        from_value=float(item['from_value']),
                        to_value=float(item['to_value']),
                        step=float(item['step']),
                    )
                )

    try:
        # CRITICAL: Call real rclpy API with descriptor - NO SIMULATION
        if 'value' in payload:
            st.node.declare_parameter(name, payload['value'], descriptor)
        else:
            if int(descriptor.type) != int(ParameterType.PARAMETER_NOT_SET):
                st.node.declare_parameter(name, Parameter.Type(int(descriptor.type)), descriptor)
            else:
                # Preserve legacy harness behavior when no explicit type is provided.
                st.node.declare_parameter(name, '', descriptor)
        result['declared'] = True

        # Echo descriptor for trace inspection (matching backend_prod)
        result['descriptor'] = _descriptor_to_json(descriptor)

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
        'byte_array': Parameter.Type.BYTE_ARRAY,
        'bool_array': Parameter.Type.BOOL_ARRAY,
        'integer_array': Parameter.Type.INTEGER_ARRAY,
        'double_array': Parameter.Type.DOUBLE_ARRAY,
        'string_array': Parameter.Type.STRING_ARRAY,
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


def exec_set_param_atomic(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute set_param_atomic operation using set_parameters_atomically()."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"set_param_atomic: missing payload in scenario {scenario_id}")

    name = payload.get('name')
    param_type = payload.get('type')
    value = payload.get('value')
    if not name:
        raise BundleError(f"set_param_atomic: payload missing 'name' in scenario {scenario_id}")
    if param_type is None:
        raise BundleError(f"set_param_atomic: payload missing 'type' in scenario {scenario_id}")
    if value is None:
        raise BundleError(f"set_param_atomic: payload missing 'value' in scenario {scenario_id}")

    type_map = {
        'string': Parameter.Type.STRING,
        'integer': Parameter.Type.INTEGER,
        'double': Parameter.Type.DOUBLE,
        'bool': Parameter.Type.BOOL,
        'byte_array': Parameter.Type.BYTE_ARRAY,
        'bool_array': Parameter.Type.BOOL_ARRAY,
        'integer_array': Parameter.Type.INTEGER_ARRAY,
        'double_array': Parameter.Type.DOUBLE_ARRAY,
        'string_array': Parameter.Type.STRING_ARRAY,
    }
    if param_type not in type_map:
        raise BundleError(f"set_param_atomic: unsupported type '{param_type}' in scenario {scenario_id}")

    result = {
        'op': 'set_param_atomic',
        'name': name,
    }
    try:
        param = Parameter(name, type_map[param_type], value)
        set_result = st.node.set_parameters_atomically([param])
        result['successful'] = bool(set_result.successful)
        if not set_result.successful and set_result.reason:
            result['reason'] = set_result.reason
    except rclpy.exceptions.ParameterNotDeclaredException:
        result['successful'] = False
        result['reason'] = 'NotDeclared'
    except Exception as e:
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
    try:
        param = st.node.get_parameter(name)
    except rclpy.exceptions.ParameterUninitializedException:
        result['set'] = False
        result['type'] = 'NOT_SET'
        return result

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
    """Register add_on_set_parameters_callback modes for callback semantics tests."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"register_param_callback: missing payload in scenario {scenario_id}")

    mode = payload.get('mode')
    if mode not in ('reject_names', 'assert_storage_old_value'):
        raise BundleError(f"register_param_callback: unsupported mode '{mode}' in scenario {scenario_id}")

    reject: List[str] = []
    reject_names = set()
    target_name: Optional[str] = None
    expected_old_value: Optional[str] = None

    if mode == 'reject_names':
        reject = payload.get('reject')
        if not isinstance(reject, list) or not all(isinstance(name, str) for name in reject):
            raise BundleError(
                f"register_param_callback: payload.reject must be a string list in scenario {scenario_id}"
            )
        reject_names = set(reject)
    else:
        target_name = payload.get('target')
        expected_old_value = payload.get('expected_old_value')
        if not isinstance(target_name, str):
            raise BundleError(
                f"register_param_callback: payload.target must be a string in scenario {scenario_id}"
            )
        if not isinstance(expected_old_value, str):
            raise BundleError(
                f"register_param_callback: payload.expected_old_value must be a string in scenario {scenario_id}"
            )

    if st.on_set_param_callback is not None:
        st.node.remove_on_set_parameters_callback(st.on_set_param_callback)
        st.on_set_param_callback = None

    def _callback(params: List[Parameter]) -> SetParametersResult:
        out = SetParametersResult()
        out.successful = True
        if mode == 'reject_names':
            for param in params:
                if param.name in reject_names:
                    out.successful = False
                    out.reason = f"RejectedByCallback:{param.name}"
                    return out
            return out

        assert target_name is not None
        assert expected_old_value is not None
        target_in_request = any(param.name == target_name for param in params)
        if not target_in_request:
            return out

        try:
            current = st.node.get_parameter(target_name)
            observed = _parameter_to_string(current)
        except rclpy.exceptions.ParameterUninitializedException:
            observed = 'NOT_SET'
        except rclpy.exceptions.ParameterNotDeclaredException:
            observed = 'NOT_SET'

        if observed != expected_old_value:
            out.successful = False
            out.reason = f"StorageNotOld:{target_name}"
        return out

    st.node.add_on_set_parameters_callback(_callback)
    st.on_set_param_callback = _callback

    result: Dict[str, Any] = {
        'op': 'register_param_callback',
        'mode': mode,
        'registered': True,
    }
    if mode == 'reject_names':
        result['reject'] = reject
    else:
        result['target'] = target_name
        result['expected_old_value'] = expected_old_value
    return result


def exec_register_pre_set_callback(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Register add_pre_set_parameters_callback with remove-name mode."""
    if not hasattr(st.node, 'add_pre_set_parameters_callback'):
        return {
            'op': 'register_pre_set_callback',
            'registered': False,
            'reason': 'UnsupportedOnThisDistro',
        }

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


def exec_get_parameter_types(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Execute get_parameter_types operation."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"get_parameter_types: missing payload in scenario {scenario_id}")

    names = payload.get('names')
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise BundleError(f"get_parameter_types: payload.names must be a string list in scenario {scenario_id}")

    result: Dict[str, Any] = {
        'op': 'get_parameter_types',
        'names': names,
        'request_count': len(names),
        'successful': True,
        'count': 0,
        'types': [],
    }
    try:
        types = st.node.get_parameter_types(names)
        result['count'] = len(types)
        result['types'] = [int(t) for t in types]
    except rclpy.exceptions.ParameterNotDeclaredException as e:
        result['successful'] = False
        result['reason'] = 'NotDeclared'
        result['reason_raw'] = str(e)
    except Exception as e:
        result['successful'] = False
        result['reason'] = f'Exception: {str(e)}'
        result['reason_raw'] = str(e)
    return result


def exec_concurrent_read_during_batch(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Probe whether a concurrent reader can observe partial non-atomic batch state."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"concurrent_read_during_batch: missing payload in scenario {scenario_id}")

    targets = payload.get('target_params')
    if not isinstance(targets, list) or not all(isinstance(name, str) for name in targets):
        raise BundleError(f"concurrent_read_during_batch: payload.target_params must be a string list in scenario {scenario_id}")

    batch_payload = payload.get('batch')
    if not isinstance(batch_payload, list):
        raise BundleError(f"concurrent_read_during_batch: payload.batch must be a list in scenario {scenario_id}")

    api = payload.get('api', 'non_atomic')
    if api not in ('non_atomic', 'atomic'):
        raise BundleError(
            f"concurrent_read_during_batch: payload.api must be 'non_atomic' or 'atomic' in scenario {scenario_id}"
        )

    trial_count = payload.get('trials', 1)
    if not isinstance(trial_count, int) or trial_count < 1:
        raise BundleError(f"concurrent_read_during_batch: payload.trials must be an integer >= 1 in scenario {scenario_id}")

    min_snapshots = payload.get('min_snapshots', 1)
    if not isinstance(min_snapshots, int) or min_snapshots < 1:
        raise BundleError(f"concurrent_read_during_batch: payload.min_snapshots must be an integer >= 1 in scenario {scenario_id}")

    reader_sleep_us = payload.get('reader_sleep_us', 50)
    if not isinstance(reader_sleep_us, int) or reader_sleep_us < 0:
        raise BundleError(
            f"concurrent_read_during_batch: payload.reader_sleep_us must be an integer >= 0 in scenario {scenario_id}"
        )

    batch = _build_parameter_list(batch_payload, scenario_id, 'concurrent_read_during_batch')

    old_params: Dict[str, Parameter] = {
        name: st.node.get_parameter(name)
        for name in targets
    }
    old_values: Dict[str, str] = {
        name: _parameter_to_string(old_params[name])
        for name in targets
    }
    new_values = dict(old_values)
    for param in batch:
        new_values[param.name] = _parameter_to_string(param)

    revert_batch = [old_params[name] for name in targets]

    successful = True
    partial_observed = False
    mixed_snapshot_count = 0
    mixed_trial_count = 0
    snapshot_count = 0

    for _ in range(trial_count):
        reset = st.node.set_parameters_atomically(revert_batch)
        if not reset.successful:
            successful = False
            break

        stop_reader = threading.Event()
        first_snapshot = threading.Event()
        snapshots: List[Dict[str, str]] = []
        snapshot_lock = threading.Lock()

        def _reader() -> None:
            while not stop_reader.is_set():
                params = st.node.get_parameters(targets)
                snap = {}
                for idx, param in enumerate(params):
                    if idx < len(targets):
                        snap[targets[idx]] = _parameter_to_string(param)
                with snapshot_lock:
                    snapshots.append(snap)
                first_snapshot.set()
                if reader_sleep_us > 0:
                    time.sleep(reader_sleep_us / 1_000_000.0)
                else:
                    time.sleep(0)

        reader_thread = threading.Thread(target=_reader, daemon=True)
        reader_thread.start()
        first_snapshot.wait(timeout=0.02)
        try:
            if api == 'atomic':
                result = st.node.set_parameters_atomically(batch)
                if not result.successful:
                    successful = False
            else:
                trial_results = st.node.set_parameters(batch)
                if not all(result.successful for result in trial_results):
                    successful = False
        finally:
            stop_reader.set()
            reader_thread.join()

        mixed_in_trial = False
        for snap in snapshots:
            old_count = 0
            new_count = 0
            for name in targets:
                value = snap.get(name)
                if value == old_values.get(name):
                    old_count += 1
                if value == new_values.get(name):
                    new_count += 1
            if old_count > 0 and new_count > 0:
                partial_observed = True
                mixed_in_trial = True
                mixed_snapshot_count += 1

        if mixed_in_trial:
            mixed_trial_count += 1

        snapshot_count += len(snapshots)

    sample_sufficient = snapshot_count >= min_snapshots
    return {
        'op': 'concurrent_read_during_batch',
        'batch_count': len(batch),
        'api': api,
        'trial_count': trial_count,
        'min_snapshots': min_snapshots,
        'reader_sleep_us': reader_sleep_us,
        'snapshot_count': snapshot_count,
        'sample_sufficient': sample_sufficient,
        'mixed_snapshot_count': mixed_snapshot_count,
        'mixed_trial_count': mixed_trial_count,
        'partial_state_observed': partial_observed,
        'successful': successful,
    }


def exec_probe_startup_declare_events(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Probe startup declaration event cardinality/signature from parameter overrides."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(f"probe_startup_declare_events: missing payload in scenario {scenario_id}")

    node_name = payload.get('node_name')
    if not isinstance(node_name, str):
        raise BundleError(
            f"probe_startup_declare_events: payload.node_name must be a string in scenario {scenario_id}"
        )

    override_specs = payload.get('parameter_overrides')
    if not isinstance(override_specs, list):
        raise BundleError(
            f"probe_startup_declare_events: payload.parameter_overrides must be a list in scenario {scenario_id}"
        )
    overrides = _build_parameter_list(override_specs, scenario_id, 'probe_startup_declare_events')
    override_names = {param.name for param in overrides}

    auto_declare = payload.get('automatically_declare_parameters_from_overrides', True)
    if not isinstance(auto_declare, bool):
        raise BundleError(
            "probe_startup_declare_events: payload.automatically_declare_parameters_from_overrides "
            f"must be bool in scenario {scenario_id}"
        )
    allow_undeclared = payload.get('allow_undeclared_parameters', False)
    if not isinstance(allow_undeclared, bool):
        raise BundleError(
            f"probe_startup_declare_events: payload.allow_undeclared_parameters must be bool in scenario {scenario_id}"
        )

    window_ms = payload.get('window_ms', 400)
    if not isinstance(window_ms, int) or window_ms < 0:
        raise BundleError(
            f"probe_startup_declare_events: payload.window_ms must be non-negative int in scenario {scenario_id}"
        )

    counts_by_event: List[int] = []
    changed_total = 0
    new_total = 0
    deleted_total = 0
    lock = threading.Lock()

    expected_1 = node_name
    expected_2 = f'/{node_name}'

    def _cb(msg: ParameterEvent) -> None:
        nonlocal changed_total, new_total, deleted_total
        src = msg.node
        if not (src == expected_1 or src == expected_2 or src.endswith(expected_2)):
            return
        new_in_scope = sum(1 for p in msg.new_parameters if p.name in override_names)
        changed_in_scope = sum(1 for p in msg.changed_parameters if p.name in override_names)
        deleted_in_scope = sum(1 for p in msg.deleted_parameters if p.name in override_names)
        keys = new_in_scope + changed_in_scope + deleted_in_scope
        if keys == 0:
            return
        with lock:
            counts_by_event.append(keys)
            changed_total += changed_in_scope
            new_total += new_in_scope
            deleted_total += deleted_in_scope

    sub = st.node.create_subscription(
        ParameterEvent,
        '/parameter_events',
        _cb,
        qos_profile_parameter_events,
    )

    probe_node = rclpy.create_node(
        node_name,
        allow_undeclared_parameters=allow_undeclared,
        automatically_declare_parameters_from_overrides=auto_declare,
        parameter_overrides=overrides,
    )
    try:
        deadline = time.monotonic() + (window_ms / 1000.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(st.node, timeout_sec=0.01)
            rclpy.spin_once(probe_node, timeout_sec=0.0)
    finally:
        try:
            st.node.destroy_subscription(sub)
        except Exception:
            pass
        probe_node.destroy_node()

    max_keys_in_single_event = max(counts_by_event) if counts_by_event else 0
    return {
        'op': 'probe_startup_declare_events',
        'node_name': node_name,
        'override_count': len(overrides),
        'window_ms': window_ms,
        'event_count': len(counts_by_event),
        'max_keys_in_single_event': max_keys_in_single_event,
        'batched_single_event_observed': max_keys_in_single_event >= len(overrides),
        'new_total': new_total,
        'changed_total': changed_total,
        'deleted_total': deleted_total,
    }


def exec_probe_startup_manual_declare_events(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Probe startup event batching for explicit declare_parameter() calls."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(
            f"probe_startup_manual_declare_events: missing payload in scenario {scenario_id}"
        )

    node_name = payload.get('node_name')
    if not isinstance(node_name, str):
        raise BundleError(
            f"probe_startup_manual_declare_events: payload.node_name must be a string in scenario {scenario_id}"
        )

    parameter_specs = payload.get('parameters')
    if not isinstance(parameter_specs, list):
        raise BundleError(
            f"probe_startup_manual_declare_events: payload.parameters must be a list in scenario {scenario_id}"
        )
    parameters = _build_parameter_list(
        parameter_specs, scenario_id, 'probe_startup_manual_declare_events'
    )
    parameter_names = {param.name for param in parameters}

    window_ms = payload.get('window_ms', 500)
    if not isinstance(window_ms, int) or window_ms < 0:
        raise BundleError(
            f"probe_startup_manual_declare_events: payload.window_ms must be non-negative int in scenario {scenario_id}"
        )

    counts_by_event: List[int] = []
    changed_total = 0
    new_total = 0
    deleted_total = 0
    lock = threading.Lock()

    expected_1 = node_name
    expected_2 = f'/{node_name}'

    def _cb(msg: ParameterEvent) -> None:
        nonlocal changed_total, new_total, deleted_total
        src = msg.node
        if not (src == expected_1 or src == expected_2 or src.endswith(expected_2)):
            return
        new_in_scope = sum(1 for p in msg.new_parameters if p.name in parameter_names)
        changed_in_scope = sum(1 for p in msg.changed_parameters if p.name in parameter_names)
        deleted_in_scope = sum(1 for p in msg.deleted_parameters if p.name in parameter_names)
        keys = new_in_scope + changed_in_scope + deleted_in_scope
        if keys == 0:
            return
        with lock:
            counts_by_event.append(keys)
            changed_total += changed_in_scope
            new_total += new_in_scope
            deleted_total += deleted_in_scope

    sub = st.node.create_subscription(
        ParameterEvent,
        '/parameter_events',
        _cb,
        qos_profile_parameter_events,
    )

    probe_node = rclpy.create_node(
        node_name,
        allow_undeclared_parameters=False,
        automatically_declare_parameters_from_overrides=False,
    )
    declare_errors: List[Dict[str, str]] = []
    successful = True
    try:
        for param in parameters:
            try:
                probe_node.declare_parameter(param.name, param.value)
            except Exception as e:
                successful = False
                declare_errors.append({'name': param.name, 'reason': str(e)})

        deadline = time.monotonic() + (window_ms / 1000.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(st.node, timeout_sec=0.01)
            rclpy.spin_once(probe_node, timeout_sec=0.0)
    finally:
        try:
            st.node.destroy_subscription(sub)
        except Exception:
            pass
        probe_node.destroy_node()

    max_keys_in_single_event = max(counts_by_event) if counts_by_event else 0
    return {
        'op': 'probe_startup_manual_declare_events',
        'node_name': node_name,
        'declare_count': len(parameters),
        'window_ms': window_ms,
        'event_count': len(counts_by_event),
        'max_keys_in_single_event': max_keys_in_single_event,
        'batched_single_event_observed': max_keys_in_single_event >= len(parameters),
        'new_total': new_total,
        'changed_total': changed_total,
        'deleted_total': deleted_total,
        'successful': successful,
        'declare_errors': declare_errors,
    }


def exec_probe_cli_typed_no_default_declare(
    w: TraceWriter,
    st: BackendState,
    scenario_id: str,
    op: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Probe typed no-default declaration using CLI override arguments."""
    payload = op.get('payload')
    if not payload:
        raise BundleError(
            f"probe_cli_typed_no_default_declare: missing payload in scenario {scenario_id}"
        )

    param_name = payload.get('param_name')
    if not isinstance(param_name, str):
        raise BundleError(
            "probe_cli_typed_no_default_declare: payload.param_name must be string "
            f"in scenario {scenario_id}"
        )

    descriptor_type = payload.get('descriptor_type')
    if not isinstance(descriptor_type, int):
        raise BundleError(
            "probe_cli_typed_no_default_declare: payload.descriptor_type must be int "
            f"in scenario {scenario_id}"
        )

    if 'cli_value' not in payload:
        raise BundleError(
            "probe_cli_typed_no_default_declare: payload.cli_value is required "
            f"in scenario {scenario_id}"
        )
    assignment = _cli_assignment_from_json_value(param_name, payload['cli_value'], scenario_id)
    node_name = payload.get('node_name', 'probe_cli_typed_no_default')
    if not isinstance(node_name, str):
        raise BundleError(
            f"probe_cli_typed_no_default_declare: payload.node_name must be string in scenario {scenario_id}"
        )

    result: Dict[str, Any] = {
        'op': 'probe_cli_typed_no_default_declare',
        'node_name': node_name,
        'param_name': param_name,
        'descriptor_type': descriptor_type,
        'cli_assignment': assignment,
        'successful': False,
        'declared': False,
        'set': False,
        'type': 'NOT_SET',
    }

    descriptor = ParameterDescriptor(type=int(descriptor_type))
    probe_node = rclpy.create_node(
        node_name,
        cli_args=['--ros-args', '-p', assignment],
        use_global_arguments=False,
        allow_undeclared_parameters=False,
        automatically_declare_parameters_from_overrides=False,
    )
    try:
        probe_node.declare_parameter(param_name, descriptor=descriptor)
        result['declared'] = bool(probe_node.has_parameter(param_name))

        param = probe_node.get_parameter(param_name)
        if param.type_ == Parameter.Type.NOT_SET:
            result['set'] = False
            result['type'] = 'NOT_SET'
        else:
            result['set'] = True
            if param.type_ == Parameter.Type.BOOL:
                result['type'] = 'bool'
                result['value'] = bool(param.value)
            elif param.type_ == Parameter.Type.INTEGER:
                result['type'] = 'integer'
                result['value'] = int(param.value)
            elif param.type_ == Parameter.Type.DOUBLE:
                result['type'] = 'double'
                result['value'] = float(param.value)
            elif param.type_ == Parameter.Type.STRING:
                result['type'] = 'string'
                result['value'] = str(param.value)
            else:
                result['type'] = 'unsupported'
                result['value'] = str(param.value)

        result['successful'] = True
    except Exception as e:
        result['successful'] = False
        result['reason'] = str(e)
    finally:
        probe_node.destroy_node()

    return result


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

    result['descriptor'] = _descriptor_to_json(descriptor)

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
