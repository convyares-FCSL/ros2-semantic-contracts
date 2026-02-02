"""Lifecycle operations for probe scenarios."""

import threading
import time
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition


class LifecycleOps:
    """Lifecycle operations implementation."""
    
    STATE_MAP = {
        "unconfigured": 1,
        "inactive": 2,
        "active": 3,
        "finalized": 4,
        "configured": 10  # Transition to inactive
    }
    
    TRANSITION_MAP = {
        "configured": Transition.TRANSITION_CONFIGURE,
        "inactive": Transition.TRANSITION_CLEANUP,
        "active": Transition.TRANSITION_ACTIVATE,
        "finalized": Transition.TRANSITION_SHUTDOWN
    }
    
    def __init__(self, node, trace, scenario_id):
        self.node = node
        self.trace = trace
        self.scenario_id = scenario_id
        self.in_transition = False
        self.transition_lock = threading.Lock()
    
    def change_state(self, payload: dict):
        """Synchronous change_state request."""
        goal_state = payload.get("goal_state")
        
        self.trace.write_event("transition_request", {"goal": goal_state}, self.scenario_id)
        
        # Check if transition already in progress (for L05 concurrent rejection)
        with self.transition_lock:
            if self.in_transition:
                self.trace.write_event("transition_response", {
                    "goal": goal_state,
                    "successful": False,
                    "code": "BUSY"
                }, self.scenario_id)
                return
        
        # Execute transition
        try:
            transition_id = self.TRANSITION_MAP.get(goal_state)
            if transition_id:
                result = self.node.trigger_configure() if goal_state == "configured" else \
                         self.node.trigger_activate() if goal_state == "active" else \
                         self.node.trigger_deactivate() if goal_state == "inactive" else \
                         self.node.trigger_shutdown()
                
                success = result == TransitionCallbackReturn.SUCCESS
                self.trace.write_event("transition_response", {
                    "goal": goal_state,
                    "successful": success
                }, self.scenario_id)
        except Exception as e:
            self.trace.write_event("transition_response", {
                "goal": goal_state,
                "successful": False,
                "error": str(e)
            }, self.scenario_id)
    
    def change_state_async(self, payload: dict):
        """Asynchronous change_state with artificial delay for L05 testing."""
        goal_state = payload.get("goal_state")
        delay_ms = payload.get("delay_ms", 0)
        
        self.trace.write_event("transition_start", {"goal": goal_state}, self.scenario_id)
        
        def slow_transition():
            with self.transition_lock:
                self.in_transition = True
            
            try:
                # Artificial delay to force concurrency window
                time.sleep(delay_ms / 1000.0)
                
                # Execute transition
                transition_id = self.TRANSITION_MAP.get(goal_state)
                if transition_id:
                    result = self.node.trigger_configure() if goal_state == "configured" else \
                             self.node.trigger_activate() if goal_state == "active" else \
                             self.node.trigger_deactivate() if goal_state == "inactive" else \
                             self.node.trigger_shutdown()
                    
                    success = result == TransitionCallbackReturn.SUCCESS
                    self.trace.write_event("transition_complete", {
                        "goal": goal_state,
                        "successful": success
                    }, self.scenario_id)
            finally:
                with self.transition_lock:
                    self.in_transition = False
        
        # Run in background thread to allow concurrent requests
        thread = threading.Thread(target=slow_transition)
        thread.daemon = True
        thread.start()
    
    def get_state(self, payload: dict):
        """Get current lifecycle state (for S11 responsiveness test)."""
        deadline_ms = payload.get("deadline_ms", 1000)
        
        start_time = time.time()
        self.trace.write_event("get_state_request", {}, self.scenario_id)
        
        try:
            # Get current state from node
            current_state = self.node.current_state[1]  # State label
            
            latency_ms = int((time.time() - start_time) * 1000)
            
            self.trace.write_event("get_state_response", {
                "state": current_state.lower() if hasattr(current_state, 'lower') else str(current_state),
                "latency_ms": latency_ms
            }, self.scenario_id)
            
            if latency_ms > deadline_ms:
                self.trace.write_event("deadline_exceeded", {
                    "deadline_ms": deadline_ms,
                    "actual_ms": latency_ms
                }, self.scenario_id)
        except Exception as e:
            self.trace.write_event("get_state_error", {"error": str(e)}, self.scenario_id)
