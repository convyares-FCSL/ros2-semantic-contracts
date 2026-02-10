"""Core backend orchestration and operation dispatch."""

import os
import rclpy
from typing import Dict, Any, Optional
from rclpy.parameter import Parameter
from rclpy.node import Node
from .trace_writer import TraceWriter
from .bundle_parser import parse_bundle
from .backend_state import BackendState
from .errors import BundleError
from .ops.clock import exec_get_clock
from .ops.generic import exec_wait
from .ops.actions import exec_send_goal
from .ops.params import (
    exec_declare_param,
    exec_set_param,
    exec_set_param_atomic,
    exec_get_param,
    exec_set_params_batch,
    exec_set_params_non_atomic,
    exec_describe_param,
    exec_undeclare_param,
    exec_register_param_callback,
    exec_register_pre_set_callback,
    exec_observe_param_events,
    exec_list_params,
    exec_get_parameter_types,
    exec_concurrent_read_during_batch,
    exec_probe_startup_declare_events,
    exec_probe_startup_manual_declare_events,
    exec_probe_cli_typed_no_default_declare,
)


# Operation handler registry
OP_HANDLERS = {
    'get_clock': exec_get_clock,
    'wait': exec_wait,
    'send_goal': exec_send_goal,
    'declare_param': exec_declare_param,
    'set_param': exec_set_param,
    'set_param_atomic': exec_set_param_atomic,
    'get_param': exec_get_param,
    'set_params_batch': exec_set_params_batch,
    'set_params_non_atomic': exec_set_params_non_atomic,
    'describe_param': exec_describe_param,
    'undeclare_param': exec_undeclare_param,
    'register_param_callback': exec_register_param_callback,
    'register_pre_set_callback': exec_register_pre_set_callback,
    'observe_param_events': exec_observe_param_events,
    'list_params': exec_list_params,
    'get_parameter_types': exec_get_parameter_types,
    'concurrent_read_during_batch': exec_concurrent_read_during_batch,
    'probe_startup_declare_events': exec_probe_startup_declare_events,
    'probe_startup_manual_declare_events': exec_probe_startup_manual_declare_events,
    'probe_cli_typed_no_default_declare': exec_probe_cli_typed_no_default_declare,
}


def _make_parameter_override(spec: Dict[str, Any], scenario_id: str) -> Parameter:
    if not isinstance(spec, dict):
        raise BundleError(
            f"scenario {scenario_id}: node_options.parameter_overrides[] must be objects"
        )
    name = spec.get('name')
    if not isinstance(name, str):
        raise BundleError(
            f"scenario {scenario_id}: node_options.parameter_overrides[].name must be a string"
        )
    if 'value' not in spec:
        raise BundleError(
            f"scenario {scenario_id}: node_options.parameter_overrides[] missing value"
        )
    value = spec['value']

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
    override_type = spec.get('type')
    if override_type is not None:
        if not isinstance(override_type, str) or override_type not in type_map:
            raise BundleError(
                f"scenario {scenario_id}: unsupported node_options override type '{override_type}'"
            )
        return Parameter(name, type_map[override_type], value)

    if isinstance(value, bool):
        return Parameter(name, Parameter.Type.BOOL, value)
    if isinstance(value, int):
        return Parameter(name, Parameter.Type.INTEGER, value)
    if isinstance(value, float):
        return Parameter(name, Parameter.Type.DOUBLE, value)
    if isinstance(value, str):
        return Parameter(name, Parameter.Type.STRING, value)

    raise BundleError(
        f"scenario {scenario_id}: cannot infer type for node_options override '{name}'"
    )


def exec_op(w: TraceWriter, st: BackendState, scenario_id: str, op_id: str, op: Dict[str, Any]) -> None:
    """Execute single operation with op_start/op_end lifecycle.
    
    Args:
        w: TraceWriter instance
        st: BackendState
        scenario_id: Scenario identifier
        op_id: Operation identifier (e.g., "H00#0")
        op: Operation dict with 'op' field
        
    Raises:
        BundleError: Unknown or unsupported operation
    """
    op_name = op.get('op')
    if not op_name:
        raise BundleError(f"Operation missing 'op' field in scenario {scenario_id}")
    
    if op_name not in OP_HANDLERS:
        raise BundleError(f"Unsupported operation: {op_name}")
    
    # Emit op_start
    w.emit({
        'type': 'op_start',
        'scenario_id': scenario_id,
        'op_id': op_id,
        'detail': {'op': op_name}
    })
    
    # Call handler
    handler = OP_HANDLERS[op_name]
    result = handler(w, st, scenario_id, op)
    
    # Emit op_end with optional result
    op_end_event = {
        'type': 'op_end',
        'scenario_id': scenario_id,
        'op_id': op_id,
    }
    if result is not None:
        op_end_event['detail'] = result
    
    w.emit(op_end_event)


def run_backend(bundle_path: str, trace_path: str) -> None:
    """Main backend orchestration.
    
    Args:
        bundle_path: Path to scenario bundle JSON
        trace_path: Path to output trace JSONL
        
    Raises:
        BundleError: Invalid bundle
        SystemError: Runtime failures
    """
    # Parse bundle
    bundle = parse_bundle(bundle_path)
    scenarios = bundle['scenarios']
    
    # Get run_id from environment (default "run_py")
    run_id = os.environ.get('BACKEND_PY_RUN_ID', 'run_py')
    
    # Create TraceWriter
    with TraceWriter(trace_path, run_id) as w:
        # Emit run_start
        w.emit({
            'type': 'run_start',
            'scenario_id': '_run',
            'detail': {'backend': 'py'}
        })
        
        # Emit backend_capabilities
        caps = [
            'actions.basic',
            'clock.ros_time',
            'ros.params.declare',
            'ros.params.set',
            'ros.params.set_atomic_single',
            'ros.params.get',
            'ros.params.set_batch',
            'ros.params.set_non_atomic',
            'ros.params.describe',
            'ros.params.delete',
            'ros.params.list',
            'ros.params.callback',
            'ros.params.observe_events',
            'ros.params.get_parameter_types',
            'ros.params.concurrent_read',
            'ros.params.startup_probe',
            'ros.params.cli_override_probe',
        ]
        if hasattr(Node, 'add_pre_set_parameters_callback'):
            caps.append('ros.params.pre_set_callback')

        w.emit({
            'type': 'backend_capabilities',
            'scenario_id': '_run',
            'detail': {
                'backend': 'py',
                'caps': caps,
                'limits': {}
            }
        })
        
        # Execute scenarios in sorted order
        for scenario_id in sorted(scenarios.keys()):
            scenario = scenarios[scenario_id]
            ops = scenario.get('ops', [])
            
            # Emit scenario_start
            w.emit({
                'type': 'scenario_start',
                'scenario_id': scenario_id,
                'detail': {'backend': 'py'}
            })
            
            # Create BackendState for this scenario
            st = BackendState()
            node_options = scenario.get('node_options', {})
            if not isinstance(node_options, dict):
                raise BundleError(
                    f"scenario {scenario_id}: node_options must be an object when provided"
                )

            allow_undeclared = node_options.get('allow_undeclared_parameters', False)
            if not isinstance(allow_undeclared, bool):
                raise BundleError(
                    f"scenario {scenario_id}: node_options.allow_undeclared_parameters must be bool"
                )
            auto_declare_overrides = node_options.get(
                'automatically_declare_parameters_from_overrides',
                False,
            )
            if not isinstance(auto_declare_overrides, bool):
                raise BundleError(
                    "scenario "
                    f"{scenario_id}: node_options.automatically_declare_parameters_from_overrides must be bool"
                )

            override_specs = node_options.get('parameter_overrides', [])
            if not isinstance(override_specs, list):
                raise BundleError(
                    f"scenario {scenario_id}: node_options.parameter_overrides must be a list"
                )
            parameter_overrides = [
                _make_parameter_override(spec, scenario_id)
                for spec in override_specs
            ]

            st.node = rclpy.create_node(
                f'oracle_backend_py_{scenario_id}',
                allow_undeclared_parameters=allow_undeclared,
                automatically_declare_parameters_from_overrides=auto_declare_overrides,
                parameter_overrides=parameter_overrides,
            )
            
            # Create ActionClient if scenario uses actions
            if any(op.get('op') == 'send_goal' for op in ops):
                from rclpy.action import ActionClient
                from test_msgs.action import Fibonacci
                st.client = ActionClient(st.node, Fibonacci, 'test_action')
            
            try:
                # Execute operations
                for idx, op in enumerate(ops):
                    op_id = f"{scenario_id}#{idx}"
                    exec_op(w, st, scenario_id, op_id, op)
            finally:
                # Cleanup state
                st.cleanup()
            
            # Emit scenario_end (no detail field)
            w.emit({
                'type': 'scenario_end',
                'scenario_id': scenario_id
            })
        
        # Emit run_end (no detail field)
        w.emit({
            'type': 'run_end',
            'scenario_id': '_run'
        })
