"""Lifecycle operations for probe scenarios."""

import time
import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

class LifecycleOps:
    """Lifecycle operations implementation using strict ROS clients."""
    
    # Standard Lifecycle Transition IDs
    TRANSITION_MAP = {
        "configured": Transition.TRANSITION_CONFIGURE,
        "inactive": Transition.TRANSITION_CLEANUP,
        "active": Transition.TRANSITION_ACTIVATE,
        "finalized": Transition.TRANSITION_INACTIVE_SHUTDOWN,
        "unconfigured": Transition.TRANSITION_CLEANUP
    }
    
    def __init__(self, harness_node, target_node, target_node_name, scenario_id):
        self.harness_node = harness_node
        self.target_node = target_node
        self.target_node_name = target_node_name
        self.scenario_id = scenario_id
        
        # Create Clients
        self.client_change_state = self.harness_node.create_client(
            ChangeState, 
            f"/{target_node_name}/change_state"
        )
        self.client_get_state = self.harness_node.create_client(
            GetState, 
            f"/{target_node_name}/get_state"
        )

    def _wait_for_service(self, client, timeout_sec=1.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            # In strict mode, we might want to fail hard, or just return false
            return False
        return True

    def change_state(self, payload: dict):
        """Synchronous change_state request."""
        goal_state = payload.get("goal_state")
        transition_id = self.TRANSITION_MAP.get(goal_state)
        
        if not transition_id:
            return {"goal": goal_state, "successful": False, "error": "Invalid goal state"}
        
        if not self._wait_for_service(self.client_change_state):
             return {"goal": goal_state, "successful": False, "error": "Service not available"}

        request = ChangeState.Request()
        request.transition.id = transition_id
        
        # Call service synchronously
        future = self.client_change_state.call_async(request)
        
        # Make a separate node for setting params? No, we need to set the param on the target node.
        # But wait, how do we set the transition delay on the target node?
        # The target node is running in the other thread. We can access it directly for configuration?
        # The prompt says: "Do NOT 'simulate' a BUSY rejection...". 
        # But we still need to tell the node to be slow (for the test).
        # Direct access to the python object to set 'transition_delay_ms' is effectively setting a parameter.
        # Ideally we'd use a parameter set, but direct property access is acceptable for *test configuration* 
        # as long as the *operation* (change_state) is done via ROS primitives.
        # BUT we don't have the node instance here anymore (we just passed names).
        # We need to pass the target node instance to set the delay.
        pass # Handle delay setting separately or pass node instance.

        # Wait for result
        rclpy.spin_until_future_complete(self.harness_node, future)
        response = future.result()
        
        return {
            "goal": goal_state,
            "successful": response.success
        }

    def change_state_async(self, payload: dict):
        """Asynchronous change_state request."""
        goal_state = payload.get("goal_state")
        delay_ms = payload.get("delay_ms", 0)
        transition_id = self.TRANSITION_MAP.get(goal_state)
        
        # Set delay on target node (Requires access to node instance!)
        # We'll update __init__ to accept the node instance just for this configuration.
        if hasattr(self, 'target_node'):
             self.target_node.transition_delay_ms = delay_ms

        if not transition_id:
             return {"goal": goal_state, "successful": False, "error": "Invalid goal"}

        request = ChangeState.Request()
        request.transition.id = transition_id
        
        # Fire and forget (Async)
        self.client_change_state.call_async(request)
        
        # We do NOT wait for it. We return immediately so the test can proceed.
        # This allows L05 concurrent requests.
        
        return {
            "goal": goal_state,
            "status": "async_started"
        }
    
    def get_state(self, payload: dict):
        """Get state with timeout (S11 check)."""
        deadline_ms = payload.get("deadline_ms", 1000)
        
        if not self._wait_for_service(self.client_get_state):
             return {"error": "GetState Service not available"}
             
        request = GetState.Request()
        future = self.client_get_state.call_async(request)
        
        start_time = time.time()
        
        # Spin with timeout
        # rclpy.spin_until_future_complete blocks. We can use the timeout_sec arg.
        timeout_sec = deadline_ms / 1000.0
        
        rclpy.spin_until_future_complete(self.harness_node, future, timeout_sec=timeout_sec)
        
        latency_ms = int((time.time() - start_time) * 1000)
        
        result_payload = {"latency_ms": latency_ms}
        
        if future.done():
            resp = future.result()
            result_payload["state"] = resp.current_state.label.lower()
        else:
            # Timeout/Deadlock detected
            result_payload["deadline_exceeded"] = True
            result_payload["deadline_ms"] = deadline_ms
            # We assume state unknown or stick to last known? 
            # The spec just wants to know if we exceeded deadline.
            result_payload["state"] = "unknown" 
            
        return result_payload

