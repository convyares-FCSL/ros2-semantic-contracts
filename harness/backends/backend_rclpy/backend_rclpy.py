#!/usr/bin/env python3
"""
Minimal rclpy backend for Oracle Harness probe scenarios.

Probe-quality implementation supporting L05, S11, P04 scenarios.
"""

import sys
import json
import argparse
from pathlib import Path

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from trace_writer import TraceWriter
from ops.lifecycle import LifecycleOps
from ops.params import ParamOps


class ProbeBackend:
    """Minimal backend for probe matrix scenarios."""
    
    def __init__(self, bundle_path: str, trace_path: str):
        self.bundle_path = Path(bundle_path)
        self.trace_path = Path(trace_path)
        self.trace = TraceWriter(trace_path)
        self.node = None
        self.executor = None
        
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
        
        # Initialize rclpy
        rclpy.init()
        
        try:
            # Process scenarios
            for scenario_id, scenario in bundle.get("scenarios", {}).items():
                self.run_scenario(scenario_id, scenario)
        finally:
            if self.executor:
                self.executor.shutdown()
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
        
        for op in ops:
            self.execute_op(op, scenario_id)
        
        self.trace.write_event("scenario_end", {"scenario_id": scenario_id}, scenario_id=scenario_id)
    
    def execute_op(self, op: dict, scenario_id: str):
        """Execute a single operation."""
        op_type = op.get("op")
        
        if op_type == "init_lifecycle_node":
            self._init_lifecycle_node(scenario_id)
        elif op_type == "wait":
            import time
            time.sleep(op.get("ms", 0) / 1000.0)
        elif op_type in ["change_state", "change_state_async", "get_state"]:
            lifecycle_ops = LifecycleOps(self.node, self.trace, scenario_id)
            getattr(lifecycle_ops, op_type)(op.get("payload", {}))
        elif op_type in ["declare_param", "set_params_batch"]:
            param_ops = ParamOps(self.node, self.trace, scenario_id)
            getattr(param_ops, op_type)(op.get("payload", {}))
        else:
            print(f"WARNING: Unsupported op: {op_type}", file=sys.stderr)
    
    def _init_lifecycle_node(self, scenario_id: str):
        """Initialize lifecycle node."""
        self.node = LifecycleNode(f"probe_node_{scenario_id}")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        
        self.trace.write_event("lifecycle_node_init", {"node_name": self.node.get_name()}, scenario_id=scenario_id)


def main():
    parser = argparse.ArgumentParser(description="rclpy backend for Oracle Harness")
    parser.add_argument("bundle_path", help="Path to scenario bundle JSON")
    parser.add_argument("trace_path", help="Path to output trace JSONL")
    
    args = parser.parse_args()
    
    backend = ProbeBackend(args.bundle_path, args.trace_path)
    sys.exit(backend.run())


if __name__ == "__main__":
    main()
