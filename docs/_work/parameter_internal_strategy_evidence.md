# Parameter Internal Strategy Evidence (Jazzy)

Purpose: record white-box evidence for parameter set-path internals so lock/atomicity claims are not inferred only from stress traces.

## rclpy (installed Jazzy package)

Source was inspected with:

```bash
python3 - <<'PY'
import inspect, rclpy.node
print(inspect.getsource(rclpy.node.Node.set_parameters))
print(inspect.getsource(rclpy.node.Node.set_parameters_atomically))
print(inspect.getsource(rclpy.node.Node._set_parameters_atomically_common))
PY
```

Observed facts:
- `Node.set_parameters` iterates the input list and calls `_set_parameters_atomically([param])` once per parameter.
- `Node.set_parameters_atomically` calls `_set_parameters_atomically(parameter_list)` once for the full batch.
- `_set_parameters_atomically_common` validates callbacks/descriptors, mutates parameter storage, and publishes parameter events.

## rclcpp (installed Jazzy headers)

Header evidence:
- `/opt/ros/jazzy/include/rclcpp/rclcpp/node_interfaces/node_parameters.hpp`
  - Declares `set_parameters(...)` and `set_parameters_atomically(...)`.
  - Contains internal `mutable std::recursive_mutex mutex_;` for node parameter state.

Note:
- Definitive lock-acquisition statements for each function body require upstream implementation source (`node_parameters.cpp`) line evidence in addition to installed headers.

## Evidence policy

For concurrency-sensitive claims:
- Always require black-box traces for observable behavior.
- Also require white-box source excerpts for internal strategy claims (locking, per-element loops, callback execution model).
