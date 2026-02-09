"""CLI entry point for backend_py."""

import sys
import rclpy
from .backend import run_backend
from .errors import BundleError, SystemError


def main():
    """Main entry point with argument parsing and exception handling."""
    # Parse arguments
    if len(sys.argv) != 3:
        print("Usage: backend_py <bundle_path> <trace_path>", file=sys.stderr)
        sys.exit(2)
    
    bundle_path = sys.argv[1]
    trace_path = sys.argv[2]
    
    # Initialize rclpy
    rclpy.init()
    
    try:
        # Run backend
        run_backend(bundle_path, trace_path)
        exit_code = 0
    except BundleError as e:
        print(f"BundleError: {e}", file=sys.stderr)
        exit_code = 2
    except SystemError as e:
        print(f"SystemError: {e}", file=sys.stderr)
        exit_code = 4
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        exit_code = 4
    finally:
        # Shutdown rclpy
        try:
            rclpy.shutdown()
        except:
            pass
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
