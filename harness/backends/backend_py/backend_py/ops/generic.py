"""Generic operation handlers."""

import time
from typing import Dict, Any, Optional
from ..trace_writer import TraceWriter
from ..backend_state import BackendState
from ..errors import BundleError


def exec_wait(w: TraceWriter, st: BackendState, scenario_id: str, op: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Execute wait operation.
    
    Args:
        w: TraceWriter instance
        st: BackendState (unused)
        scenario_id: Scenario identifier
        op: Operation dict with 'ms' field
        
    Returns:
        None (no result)
        
    Raises:
        BundleError: Missing 'ms' field
    """
    if 'ms' not in op:
        raise BundleError(f"wait op missing 'ms' field in scenario {scenario_id}")
    
    ms = op['ms']
    time.sleep(ms / 1000.0)
    
    return None
