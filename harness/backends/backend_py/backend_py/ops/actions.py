"""Action operation handlers."""

import rclpy
from typing import Dict, Any, Optional
from ..trace_writer import TraceWriter
from ..backend_state import BackendState
from ..errors import BundleError, SystemError


def exec_send_goal(w: TraceWriter, st: BackendState, scenario_id: str, op: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Execute send_goal operation.
    
    Creates action client, sends goal, emits goal_send_decision event.
    
    Args:
        w: TraceWriter instance
        st: BackendState with initialized client
        scenario_id: Scenario identifier
        op: Operation dict with 'goal_id' field
        
    Returns:
        None (no result)
        
    Raises:
        BundleError: Missing 'goal_id' field
        SystemError: Action server unavailable or timeout
    """
    if 'goal_id' not in op:
        raise BundleError(f"send_goal op missing 'goal_id' field in scenario {scenario_id}")
    
    goal_id = op['goal_id']
    
    # Wait for action server
    if not st.client.wait_for_server(timeout_sec=2.0):
        raise SystemError(f"Action server not available for goal {goal_id}")
    
    # Import here to avoid dependency on test_msgs at module level
    from test_msgs.action import Fibonacci
    
    # Create goal message (dummy order value)
    goal_msg = Fibonacci.Goal()
    goal_msg.order = 1
    
    # Send goal asynchronously
    send_goal_future = st.client.send_goal_async(goal_msg)
    
    # CRITICAL: Single-threaded spin (no executor thread)
    rclpy.spin_until_future_complete(st.node, send_goal_future, timeout_sec=2.0)
    
    # Check future completed
    if not send_goal_future.done():
        raise SystemError(f"send_goal timeout for goal {goal_id}")
    
    # Extract goal handle and acceptance status
    goal_handle = send_goal_future.result()
    accepted = goal_handle.accepted if goal_handle else False
    
    # Store goal handle for future ops (e.g., cancel_goal)
    if accepted and goal_handle:
        st.goal_handles[goal_id] = goal_handle
    
    # Emit goal_send_decision event
    w.emit({
        'type': 'goal_send_decision',
        'scenario_id': scenario_id,
        'goal_id': goal_id,
        'accepted': accepted
    })
    
    return None
