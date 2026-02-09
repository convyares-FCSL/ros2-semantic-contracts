"""Exception types for backend_py exit code mapping."""


class BundleError(Exception):
    """Bundle validation or configuration error (exit code 2)."""
    pass


class SystemError(Exception):
    """Runtime or ROS system error (exit code 4)."""
    pass
