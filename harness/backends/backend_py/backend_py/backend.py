"""Core backend orchestration and operation dispatch."""

import os
import rclpy
from typing import Dict, Any, Optional
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
    exec_get_param,
    exec_set_params_batch,
    exec_describe_param,
    exec_undeclare_param,
)


# Operation handler registry
OP_HANDLERS = {
    'get_clock': exec_get_clock,
    'wait': exec_wait,
    'send_goal': exec_send_goal,
    'declare_param': exec_declare_param,
    'set_param': exec_set_param,
    'get_param': exec_get_param,
    'set_params_batch': exec_set_params_batch,
    'describe_param': exec_describe_param,
    'undeclare_param': exec_undeclare_param,
}


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
        w.emit({
            'type': 'backend_capabilities',
            'scenario_id': '_run',
            'detail': {
                'backend': 'py',
                'caps': [
                    'actions.basic',
                    'clock.ros_time',
                    'ros.params.declare',
                    'ros.params.set',
                    'ros.params.get',
                    'ros.params.set_batch',
                    'ros.params.describe',
                    'ros.params.delete',
                ],
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
            st.node = rclpy.create_node(
                f'oracle_backend_py_{scenario_id}',
                allow_undeclared_parameters=False,
                automatically_declare_parameters_from_overrides=False
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
