"""Clock operation handlers."""

import random
import rclpy
from typing import Dict, Any, Optional
from ..trace_writer import TraceWriter
from ..backend_state import BackendState


def exec_get_clock(w: TraceWriter, st: BackendState, scenario_id: str, op: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Execute get_clock operation.
    
    Creates temporary node, reads ROS clock, returns timestamp.
    
    Args:
        w: TraceWriter instance
        st: BackendState (unused for this op)
        scenario_id: Scenario identifier
        op: Operation dict
        
    Returns:
        Result dict with timestamp_ns and clock_type
    """
    # Create temporary node for clock access
    node_name = f'oracle_clock_node_{random.randint(0, 999999)}'
    node = rclpy.create_node(node_name)
    
    try:
        # Get ROS time
        now = node.get_clock().now()
        timestamp_ns = now.nanoseconds
        
        # Return result (included in op_end.detail by dispatcher)
        return {
            'op': 'get_clock',
            'timestamp_ns': timestamp_ns,
            'clock_type': 'ros_time'
        }
    finally:
        node.destroy_node()
