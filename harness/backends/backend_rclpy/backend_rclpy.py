#!/usr/bin/env python3
"""
Minimal rclpy backend for Oracle Harness probe scenarios.

Probe-quality implementation supporting L05, S11, P04 scenarios.
"""

import sys
import json
import argparse
from pathlib import Path

import os
import threading
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

import time

from trace_writer import TraceWriter
from ops.lifecycle import LifecycleOps
from ops.params import ParamOps


class SlowLifecycleNode(LifecycleNode):
    """
    Lifecycle node that can simulate slow transitions.
    
    This uses time.sleep() in transition callbacks to authenticate 'busy' states 
    for L05 (Concurrent Transition) and S11 (Lifecycle Safety). This is preferred 
    over artificial harness delays because it blocks the actual ROS lifecycle 
    executor thread, validating that the node correctly handles (or rejects) 
    incoming requests while busy.
    """
    
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.transition_delay_ms = 0

    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        if self.transition_delay_ms > 0:
            time.sleep(self.transition_delay_ms / 1000.0)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        if self.transition_delay_ms > 0:
            time.sleep(self.transition_delay_ms / 1000.0)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        if self.transition_delay_ms > 0:
            time.sleep(self.transition_delay_ms / 1000.0)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        if self.transition_delay_ms > 0:
            time.sleep(self.transition_delay_ms / 1000.0)
        return TransitionCallbackReturn.SUCCESS


class ProbeBackend:
    """Minimal backend for probe matrix scenarios."""
    
    def __init__(self, bundle_path: str, trace_path: str):
        self.bundle_path = Path(bundle_path)
        self.trace_path = Path(trace_path)
        self.trace = TraceWriter(trace_path)
        self.node = None
        self.harness_node = None
        self.executor = None
        self.lifecycle_ops = None
        
    def load_bundle(self) -> dict:
        """Load scenario bundle from JSON file."""
        try:
            with open(self.bundle_path) as f:
                return json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            print(f"ERROR: Failed to load bundle: {e}", file=sys.stderr)
            sys.exit(2)
    
    def run(self) -> int:
        """Execute bundle scenarios and emit trace."""
        bundle = self.load_bundle()
        
        # Emit run_start
        self.trace.write_event("run_start", {"backend": "rclpy"}, scenario_id="_run")

        # Emit backend_capabilities
        self.trace.write_event("backend_capabilities", {
            "backend": "rclpy",
            "caps": [
                "ros.lifecycle",
                "ros.params.declare",
                "ros.params.set",
                "ros.params.set_batch"
            ],
            "limits": {}
        }, scenario_id="_run")
        
        # Initialize rclpy
        rclpy.init()
        
        try:
            # Process scenarios
            for scenario_id, scenario in bundle.get("scenarios", {}).items():
                self.run_scenario(scenario_id, scenario)
        finally:
            if self.executor:
                self.executor.shutdown()
                # self.executor_thread.join() # Daemon thread, let it die
            if self.harness_node:
                self.harness_node.destroy_node()
            if self.node:
                self.node.destroy_node()
            rclpy.shutdown()
        
        # Emit run_end
        self.trace.write_event("run_end", {}, scenario_id="_run")
        self.trace.close()
        
        return 0
    
    def run_scenario(self, scenario_id: str, scenario: dict):
        """Execute a single scenario."""
        self.trace.write_event("scenario_start", {"scenario_id": scenario_id}, scenario_id=scenario_id)
        
        ops = scenario.get("ops", [])
        
        for i, op in enumerate(ops):
            op_id = f"{scenario_id}#{i}"
            self.execute_op(op, scenario_id, op_id)
        
        self.trace.write_event("scenario_end", {"scenario_id": scenario_id}, scenario_id=scenario_id)
    
    def execute_op(self, op: dict, scenario_id: str, op_id: str):
        """Execute a single operation."""
        op_type = op.get("op")
        payload = op.get("payload", {})
        
        self.trace.write_event("op_start", {
            "op": op_type,
            "payload": payload
        }, scenario_id=scenario_id, op_id=op_id)
        
        result = {}
        try:
            if op_type == "init_lifecycle_node":
                self._init_lifecycle_node(scenario_id)
            elif op_type == "wait":
                import time
                time.sleep(op.get("ms", 0) / 1000.0)
            elif op_type in ["change_state", "change_state_async", "get_state"]:
                if self.lifecycle_ops:
                    result = getattr(self.lifecycle_ops, op_type)(payload) or {}
                else:
                    print("ERROR: LifecycleOps not initialized (call init_lifecycle_node first)", file=sys.stderr)
                    result = {"successful": False, "error": "LifecycleOps not initialized"}
            elif op_type in ["declare_param", "set_params_batch"]:
                if not self.node:
                    self._init_lifecycle_node(scenario_id)
                param_ops = ParamOps(self.node, scenario_id)
                result = getattr(param_ops, op_type)(payload) or {}
            else:
                print(f"WARNING: Unsupported op: {op_type}", file=sys.stderr)
                result = {"error": f"Unsupported op {op_type}"}
        except Exception as e:
            result = {"successful": False, "error": str(e)}
        finally:
            self.trace.write_event("op_end", {"result": result}, scenario_id=scenario_id, op_id=op_id)
    
    def _init_lifecycle_node(self, scenario_id: str):
        """Initialize lifecycle node."""
        if self.node:
             return
        
        node_name = f"probe_node_{scenario_id}"
        self.node = SlowLifecycleNode(node_name)
        
        fidelity = os.environ.get("RCLPY_FIDELITY_MODE", "strict")
        
        if fidelity == "strict":
             self.executor = rclpy.executors.SingleThreadedExecutor()
        else:
             # Allow loose mode if explicitly requested (though mostly deprecated by this refactor)
             self.executor = MultiThreadedExecutor()
             
        self.executor.add_node(self.node)
        
        # Spin executor in background thread so blocking callbacks don't block harness
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        
        # Initialize ops with clients instead of direct node access
        # We need a temporary node to creates clients? 
        # No, we can create clients on the harness side if we had a harness node.
        # But we don't have a harness node. We are using the internal node.
        # Wait, if we use the internal node to create clients to ITSELF, requests might fail if single threaded?
        # A node calling a service on itself in SingleThreadedExecutor WILL DEADLOCK if synchronous.
        # Asynchronous calls are fine if handled properly.
        # BUT, to be safer and cleaner, we should probably have a separate "Harness Node" for valid client behavior?
        # The prompt says: "Use rclpy.executors.SingleThreadedExecutor() for ALL lifecycle tests".
        # If we add a harness node to the SAME executor, it's fine.
        # If we use a separate executor for the harness node, even better.
        
        # Let's create a separate Harness Node for sending requests.
        self.harness_node = rclpy.create_node(f"harness_{scenario_id}")
        self.lifecycle_ops = LifecycleOps(self.harness_node, self.node, node_name, scenario_id)


def main():
    parser = argparse.ArgumentParser(description="rclpy backend for Oracle Harness")
    parser.add_argument("bundle_path", help="Path to scenario bundle JSON")
    parser.add_argument("trace_path", help="Path to output trace JSONL")
    
    args = parser.parse_args()
    
    backend = ProbeBackend(args.bundle_path, args.trace_path)
    sys.exit(backend.run())


if __name__ == "__main__":
    main()
