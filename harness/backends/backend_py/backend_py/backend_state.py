"""Per-scenario ROS 2 state container."""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable, List
import rclpy.node


@dataclass
class BackendState:
    """Container for per-scenario ROS 2 resources.
    
    Minimal fields for H00/H01 support.
    """
    node: Optional[rclpy.node.Node] = None
    client: Optional[Any] = None  # ActionClient for H00
    goal_handles: Dict[str, Any] = field(default_factory=dict)
    param_event_sub: Optional[Any] = None
    param_event_buffer: List[Dict[str, Any]] = field(default_factory=list)
    on_set_param_callback: Optional[Callable[..., Any]] = None
    pre_set_param_callback: Optional[Callable[..., Any]] = None
    
    def cleanup(self) -> None:
        """Destroy ROS 2 resources."""
        if self.node:
            if self.on_set_param_callback is not None:
                try:
                    self.node.remove_on_set_parameters_callback(self.on_set_param_callback)
                except Exception:
                    pass
                self.on_set_param_callback = None

            if self.pre_set_param_callback is not None:
                try:
                    self.node.remove_pre_set_parameters_callback(self.pre_set_param_callback)
                except Exception:
                    pass
                self.pre_set_param_callback = None

            self.node.destroy_node()
            self.node = None
