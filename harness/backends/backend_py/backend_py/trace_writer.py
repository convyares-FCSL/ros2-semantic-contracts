"""Thread-safe JSONL trace emission with envelope fields."""

import json
import time
import threading
from typing import Dict, Any, Optional


class TraceWriter:
    """Thread-safe JSONL trace writer with envelope field injection."""
    
    def __init__(self, trace_path: str, run_id: str):
        """Initialize trace writer.
        
        Args:
            trace_path: Path to output trace file
            run_id: Run identifier for envelope
        """
        self.file = open(trace_path, 'w', buffering=1)  # Line-buffered
        self.run_id = run_id
        self.t0 = time.monotonic()
        self.sequence = 0
        self.lock = threading.Lock()
    
    def emit(self, event: Dict[str, Any]) -> None:
        """Emit event with envelope fields.
        
        Adds version, run_id, t_ns, and sequence to event dict.
        Thread-safe via lock.
        
        Args:
            event: Event dict (modified in-place)
        """
        with self.lock:
            # Calculate timestamp in nanoseconds
            t_ns = int((time.monotonic() - self.t0) * 1e9)
            
            # Add envelope fields
            event['version'] = '0.1'
            event['run_id'] = self.run_id
            event['t_ns'] = t_ns
            event['sequence'] = self.sequence
            self.sequence += 1
            
            # Write compact JSON + newline
            json.dump(event, self.file, separators=(',', ':'))
            self.file.write('\n')
            self.file.flush()
    
    def close(self) -> None:
        """Flush and close trace file."""
        if self.file:
            self.file.flush()
            self.file.close()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
