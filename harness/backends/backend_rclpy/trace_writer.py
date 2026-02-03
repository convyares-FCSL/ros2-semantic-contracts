"""JSONL trace writer for Oracle Harness schema compliance."""

import json
import time
from pathlib import Path
from threading import Lock


class TraceWriter:
    """Write oracle trace events to JSONL file."""
    
    def __init__(self, trace_path: str):
        self.trace_path = Path(trace_path)
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(self.trace_path, 'w')
        self.sequence = 0
        self.start_time = time.time_ns()
        self.lock = Lock()
    
    def write_event(self, event_type: str, detail: dict, scenario_id: str = "_run", **kwargs):
        """Write a single trace event in oracle schema format."""
        with self.lock:
            event = {
                "version": "0.1",
                "run_id": "run_rclpy",
                "t_ns": time.time_ns() - self.start_time,
                "sequence": self.sequence,
                "type": event_type,
                "scenario_id": scenario_id,
                "detail": detail
            }
            # Merge optional fields (e.g. op_id)
            event.update(kwargs)
            self.file.write(json.dumps(event) + '\n')
            self.file.flush()
            self.sequence += 1
    
    def close(self):
        """Close trace file."""
        self.file.close()
