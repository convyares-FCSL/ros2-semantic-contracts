"""Per-scenario ROS 2 state container."""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import rclpy.node


@dataclass
class BackendState:
    """Container for per-scenario ROS 2 resources.
    
    Minimal fields for H00/H01 support.
    """
    node: Optional[rclpy.node.Node] = None
    client: Optional[Any] = None  # ActionClient for H00
    goal_handles: Dict[str, Any] = field(default_factory=dict)
    
    def cleanup(self) -> None:
        """Destroy ROS 2 resources."""
        if self.node:
            self.node.destroy_node()
            self.node = None
