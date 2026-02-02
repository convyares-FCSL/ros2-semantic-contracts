"""Parameter operations for probe scenarios."""

from rclpy.parameter import Parameter


class ParamOps:
    """Parameter operations implementation."""
    
    def __init__(self, node, trace, scenario_id):
        self.node = node
        self.trace = trace
        self.scenario_id = scenario_id
    
    def declare_param(self, payload: dict):
        """Declare a parameter."""
        param_name = payload.get("name")
        
        self.trace.write_event("param_declare_request", {"name": param_name}, self.scenario_id)
        
        try:
            # Declare parameter (will fail if already declared)
            self.node.declare_parameter(param_name, None)
            
            self.trace.write_event("param_declare_response", {
                "name": param_name,
                "successful": True
            }, self.scenario_id)
        except Exception as e:
            # Redeclaration or other error
            self.trace.write_event("param_declare_response", {
                "name": param_name,
                "successful": False,
                "error": str(e)
            }, self.scenario_id)
    
    def set_params_batch(self, payload: dict):
        """Set multiple parameters atomically (for P04 test)."""
        params = payload.get("params", [])
        
        self.trace.write_event("param_set_batch_request", {
            "param_count": len(params)
        }, self.scenario_id)
        
        try:
            # Build parameter list
            param_list = []
            for p in params:
                param_name = p.get("name")
                param_value = p.get("value")
                param_type = p.get("type")
                
                # Convert type string to Python type
                if param_type == "string":
                    value = str(param_value)
                elif param_type == "integer":
                    value = int(param_value)
                elif param_type == "double":
                    value = float(param_value)
                elif param_type == "bool":
                    value = bool(param_value)
                else:
                    value = param_value
                
                param_list.append(Parameter(param_name, Parameter.Type.from_parameter_value(value), value))
            
            # Attempt batch set (should be atomic - all or nothing)
            results = self.node.set_parameters(param_list)
            
            # Check if all succeeded
            all_successful = all(r.successful for r in results)
            
            self.trace.write_event("param_set_batch_response", {
                "all_successful": all_successful,
                "results": [{"successful": r.successful, "reason": r.reason} for r in results]
            }, self.scenario_id)
            
        except Exception as e:
            # Batch failed entirely
            self.trace.write_event("param_set_batch_response", {
                "all_successful": False,
                "error": str(e)
            }, self.scenario_id)
