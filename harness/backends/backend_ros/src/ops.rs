use crate::{
    bundle::{Op, Scenario},
    error::BackendError,
    io::EventWriter,
    state::BackendState,
};

use serde_json::json;
use std::time::Duration;

use rclrs::vendor::rcl_interfaces::{
    msg::{ParameterDescriptor, ParameterValue},
    srv::{DescribeParameters, SetParameters, SetParametersAtomically},
};

pub fn run_scenario(
    w: &mut EventWriter,
    id: &str,
    scenario: &Scenario,
    set_param_client: &rclrs::Client<SetParameters>,
    describe_param_client: &rclrs::Client<DescribeParameters>,
    set_param_atomically_client: &rclrs::Client<SetParametersAtomically>,
) -> Result<(), BackendError> {
    let mut st = BackendState::default();

    w.emit(json!({
        "type": "scenario_start",
        "scenario_id": id,
        "detail": { "backend": "ros" }
    }))?;

    for (i, op) in scenario.ops.iter().enumerate() {
        let op_id = format!("{id}#{i}");
        exec_op(
            w,
            &mut st,
            id,
            &op_id,
            op,
            set_param_client,
            describe_param_client,
            set_param_atomically_client,
        )?;
    }

    w.emit(json!({
        "type": "scenario_end",
        "scenario_id": id,
        "detail": { "backend": "ros" }
    }))?;

    Ok(())
}

fn exec_op(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op_id: &str,
    op: &Op,
    set_param_client: &rclrs::Client<SetParameters>,
    describe_param_client: &rclrs::Client<DescribeParameters>,
    set_param_atomically_client: &rclrs::Client<SetParametersAtomically>,
) -> Result<(), BackendError> {
    w.emit(json!({ "type": "op_start", "scenario_id": scenario_id, "op_id": op_id, "detail": { "op": op.op } }))?;

    match op.op.as_str() {
        "send_goal" => exec_send_goal(w, scenario_id, op)?,
        "wait" => exec_wait(op)?,
        "complete_terminal" => exec_complete_terminal(w, st, scenario_id, op)?,
        "attempt_terminal_override" => exec_attempt_terminal_override(w, st, scenario_id, op)?,

        "set_param" => exec_set_param(w, scenario_id, op, set_param_client)?,
        "set_params_batch" => {
            exec_set_params_batch(w, scenario_id, op, set_param_atomically_client)?
        }
        "describe_param" => exec_describe_param(w, scenario_id, op, describe_param_client)?,
        "declare_param" => exec_declare_param(w, st, scenario_id, op)?,

        other => {
            w.emit(json!({
                "type": "diagnostic",
                "scenario_id": scenario_id,
                "detail": { "error": { "kind": "unsupported_op", "op": other } }
            }))?;
            return Err(BackendError::usage(format!("unsupported op: {other}")));
        }
    }

    w.emit(json!({ "type": "op_end", "scenario_id": scenario_id, "op_id": op_id }))?;

    Ok(())
}

fn exec_set_param(
    w: &mut EventWriter,
    scenario_id: &str,
    op: &Op,
    client: &rclrs::Client<SetParameters>,
) -> Result<(), BackendError> {
    let payload = op
        .payload
        .as_ref()
        .ok_or_else(|| BackendError::usage("set_param missing payload"))?;

    let name = payload
        .get("name")
        .and_then(|v| v.as_str())
        .ok_or_else(|| BackendError::usage("set_param payload.name missing"))?;

    let val_json = payload
        .get("value")
        .ok_or_else(|| BackendError::usage("set_param payload.value missing"))?;

    let type_str = payload
        .get("type")
        .and_then(|v| v.as_str())
        .unwrap_or("string");

    // Map to ParameterValue (typed evidence for service call).
    let mut pval = ParameterValue::default();
    match type_str {
        "bool" | "boolean" => {
            pval.type_ = 1; // PARAMETER_BOOL
            pval.bool_value = val_json.as_bool().unwrap_or(false);
        }
        "int" | "integer" => {
            pval.type_ = 2; // PARAMETER_INTEGER
            pval.integer_value = val_json.as_i64().unwrap_or(0);
        }
        "double" | "float" => {
            pval.type_ = 3; // PARAMETER_DOUBLE
            pval.double_value = val_json.as_f64().unwrap_or(0.0);
        }
        "string" | _ => {
            pval.type_ = 4; // PARAMETER_STRING
            pval.string_value = match val_json {
                serde_json::Value::String(s) => s.clone(),
                _ => val_json.to_string(),
            };
        }
    }

    w.emit(json!({
        "type": "param_set_request",
        "scenario_id": scenario_id,
        "node": "/oracle_backend_ros",
        "name": name,
        "value": val_json
    }))?;

    let request = rclrs::vendor::rcl_interfaces::srv::SetParameters_Request {
        parameters: vec![rclrs::vendor::rcl_interfaces::msg::Parameter {
            name: name.to_string(),
            value: pval,
        }],
    };

    let future: rclrs::Promise<(
        rclrs::vendor::rcl_interfaces::srv::SetParameters_Response,
        rclrs::ServiceInfo,
    )> = client
        .call(&request)
        .map_err(|e| BackendError::system(e).context("failed to call set_parameters"))?;

    // Block on response (spinner thread is running).
    let result = futures::executor::block_on(future);

    match result {
        Ok((response, _info)) => {
            let r = response
                .results
                .first()
                .ok_or_else(|| BackendError::system(anyhow::anyhow!("empty response results")))?;

            w.emit(json!({
                "type": "param_set_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "name": name,
                "successful": r.successful,
                "reason": r.reason
            }))?;
        }
        Err(e) => {
            w.emit(json!({
                "type": "param_set_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "name": name,
                "successful": null,
                "reason": format!("service_call_failed: {}", e)
            }))?;
        }
    }

    Ok(())
}

fn exec_set_params_batch(
    w: &mut EventWriter,
    scenario_id: &str,
    op: &Op,
    client: &rclrs::Client<SetParametersAtomically>,
) -> Result<(), BackendError> {
    let payload = op
        .payload
        .as_ref()
        .ok_or_else(|| BackendError::usage("set_params_batch missing payload"))?;

    let params_arr = payload
        .get("params")
        .and_then(|v| v.as_array())
        .ok_or_else(|| {
            BackendError::usage("set_params_batch payload.params missing or not array")
        })?;

    let mut ros_params: Vec<rclrs::vendor::rcl_interfaces::msg::Parameter> = Vec::new();
    let mut names: Vec<String> = Vec::new();

    for p in params_arr {
        let name = p
            .get("name")
            .and_then(|v| v.as_str())
            .ok_or_else(|| BackendError::usage("set_params_batch param missing name"))?;
        let val_json = p
            .get("value")
            .ok_or_else(|| BackendError::usage("set_params_batch param missing value"))?;
        let type_str = p.get("type").and_then(|v| v.as_str()).unwrap_or("string");

        let mut pval = ParameterValue::default();
        match type_str {
            "bool" | "boolean" => {
                pval.type_ = 1;
                pval.bool_value = val_json.as_bool().unwrap_or(false);
            }
            "int" | "integer" => {
                pval.type_ = 2;
                pval.integer_value = val_json.as_i64().unwrap_or(0);
            }
            "double" | "float" => {
                pval.type_ = 3;
                pval.double_value = val_json.as_f64().unwrap_or(0.0);
            }
            _ => {
                pval.type_ = 4;
                pval.string_value = match val_json {
                    serde_json::Value::String(s) => s.clone(),
                    _ => val_json.to_string(),
                };
            }
        }

        names.push(name.to_string());
        ros_params.push(rclrs::vendor::rcl_interfaces::msg::Parameter {
            name: name.to_string(),
            value: pval,
        });
    }

    w.emit(json!({
        "type": "param_set_batch_request",
        "scenario_id": scenario_id,
        "node": "/oracle_backend_ros",
        "names": names
    }))?;

    let request = rclrs::vendor::rcl_interfaces::srv::SetParametersAtomically_Request {
        parameters: ros_params,
    };

    let future: rclrs::Promise<(
        rclrs::vendor::rcl_interfaces::srv::SetParametersAtomically_Response,
        rclrs::ServiceInfo,
    )> = client
        .call(&request)
        .map_err(|e| BackendError::system(e).context("failed to call set_parameters_atomically"))?;

    let result = futures::executor::block_on(future);

    match result {
        Ok((response, _info)) => {
            let result = response.result;
            w.emit(json!({
                "type": "param_set_batch_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "names": names,
                "all_successful": result.successful,
                "reason": result.reason
            }))?;
        }
        Err(e) => {
            w.emit(json!({
                "type": "param_set_batch_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "names": names,
                "all_successful": null,
                "reason": format!("service_call_failed: {}", e)
            }))?;
        }
    }

    Ok(())
}

fn exec_describe_param(
    w: &mut EventWriter,
    scenario_id: &str,
    op: &Op,
    client: &rclrs::Client<DescribeParameters>,
) -> Result<(), BackendError> {
    let payload = op
        .payload
        .as_ref()
        .ok_or_else(|| BackendError::usage("describe_param missing payload"))?;

    let name = payload
        .get("name")
        .and_then(|v| v.as_str())
        .ok_or_else(|| BackendError::usage("describe_param payload.name missing"))?;

    w.emit(json!({
        "type": "param_describe_request",
        "scenario_id": scenario_id,
        "node": "/oracle_backend_ros",
        "name": name
    }))?;

    let request = rclrs::vendor::rcl_interfaces::srv::DescribeParameters_Request {
        names: vec![name.to_string()],
    };

    let future: rclrs::Promise<(
        rclrs::vendor::rcl_interfaces::srv::DescribeParameters_Response,
        rclrs::ServiceInfo,
    )> = client
        .call(&request)
        .map_err(|e| BackendError::system(e).context("failed to call describe_parameters"))?;

    let result = futures::executor::block_on(future);

    match result {
        Ok((response, _info)) => {
            // Deterministic ordering: make ordering explicit even if ROS changes internal behavior.
            let mut descriptors: Vec<serde_json::Value> = response
                .descriptors
                .iter()
                .map(parameter_descriptor_to_json)
                .collect();

            descriptors.sort_by(|a, b| {
                let an = a.get("name").and_then(|v| v.as_str()).unwrap_or("");
                let bn = b.get("name").and_then(|v| v.as_str()).unwrap_or("");
                an.cmp(bn)
            });

            // P08: Emit param_type for unknown/known parameters.
            // If the parameter is unknown, ROS returns a descriptor with type 0 (NOT_SET).
            let param_type_str = response
                .descriptors
                .first()
                .map(|d| param_type_to_string(d.type_))
                .unwrap_or("NOT_SET");

            w.emit(json!({
                "type": "param_describe_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "name": name,
                "param_type": param_type_str,
                "detail": {
                    "descriptors": descriptors
                }
            }))?;
        }
        Err(e) => {
            w.emit(json!({
                "type": "param_describe_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "name": name,
                "detail": {
                    "error": format!("service_call_failed: {}", e)
                }
            }))?;
        }
    }

    Ok(())
}

fn exec_declare_param(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op: &Op,
) -> Result<(), BackendError> {
    let payload = op
        .payload
        .as_ref()
        .ok_or_else(|| BackendError::usage("declare_param missing payload"))?;

    let name = payload
        .get("name")
        .and_then(|v| v.as_str())
        .ok_or_else(|| BackendError::usage("declare_param payload.name missing"))?;

    w.emit(json!({
        "type": "param_declare_request",
        "scenario_id": scenario_id,
        "node": "/oracle_backend_ros",
        "name": name
    }))?;

    // Check if already declared (redeclaration detection).
    if st.declared_params.contains(name) {
        w.emit(json!({
            "type": "param_declare_response",
            "scenario_id": scenario_id,
            "node": "/oracle_backend_ros",
            "name": name,
            "successful": false,
            "reason": "already_declared"
        }))?;
    } else {
        st.declared_params.insert(name.to_string());
        w.emit(json!({
            "type": "param_declare_response",
            "scenario_id": scenario_id,
            "node": "/oracle_backend_ros",
            "name": name,
            "successful": true
        }))?;
    }

    Ok(())
}

fn parameter_descriptor_to_json(d: &ParameterDescriptor) -> serde_json::Value {
    json!({
        "name": d.name,
        "type": d.type_,
        "description": d.description,
        "additional_constraints": d.additional_constraints,
        "read_only": d.read_only,
        "dynamic_typing": d.dynamic_typing,
        "floating_point_range": d.floating_point_range.iter().map(|r| json!({
            "from_value": r.from_value,
            "to_value": r.to_value,
            "step": r.step
        })).collect::<Vec<_>>(),
        "integer_range": d.integer_range.iter().map(|r| json!({
            "from_value": r.from_value,
            "to_value": r.to_value,
            "step": r.step
        })).collect::<Vec<_>>(),
    })
}

fn param_type_to_string(t: u8) -> &'static str {
    match t {
        1 => "bool",
        2 => "integer",
        3 => "double",
        4 => "string",
        5 => "byte_array",
        6 => "bool_array",
        7 => "integer_array",
        8 => "double_array",
        9 => "string_array",
        _ => "NOT_SET",
    }
}

fn exec_send_goal(w: &mut EventWriter, scenario_id: &str, op: &Op) -> Result<(), BackendError> {
    let goal_id = op
        .goal_id
        .as_deref()
        .ok_or_else(|| BackendError::usage("send_goal missing goal_id"))?;

    let mode = op
        .payload
        .as_ref()
        .and_then(|p| p.get("mode"))
        .and_then(|m| m.as_str())
        .ok_or_else(|| BackendError::usage("send_goal missing payload.mode"))?;

    w.emit(json!({
        "type": "goal_send",
        "scenario_id": scenario_id,
        "goal_id": goal_id
    }))?;

    let accepted = match mode {
        "accept" => true,
        "reject" => false,
        _ => return Err(BackendError::usage(format!("invalid mode: {mode}"))),
    };

    let mut ev = json!({
        "type": "goal_response",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "accepted": accepted
    });

    if !accepted {
        ev.as_object_mut()
            .unwrap()
            .insert("reason".into(), json!("ros_scaffold_reject"));
    }

    w.emit(ev)?;
    Ok(())
}

fn exec_complete_terminal(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op: &Op,
) -> Result<(), BackendError> {
    let goal_id = op
        .goal_id
        .as_deref()
        .ok_or_else(|| BackendError::usage("complete_terminal missing goal_id"))?;

    let status = op
        .status
        .as_deref()
        .ok_or_else(|| BackendError::usage("complete_terminal missing status"))?;

    let attempt = st.terminal_attempts.entry(goal_id.to_string()).or_insert(0);
    *attempt += 1;

    if st.terminal.contains_key(goal_id) {
        w.emit(json!({
            "type": "terminal_set_attempt",
            "scenario_id": scenario_id,
            "goal_id": goal_id,
            "attempt": *attempt,
            "allowed": false,
            "reason": "terminal_immutable"
        }))?;
        return Ok(());
    }

    st.terminal.insert(goal_id.to_string(), status.to_string());

    w.emit(json!({
        "type": "status",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "status": status
    }))?;

    w.emit(json!({
        "type": "result",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "status": status
    }))?;

    Ok(())
}

fn exec_attempt_terminal_override(
    w: &mut EventWriter,
    st: &mut BackendState,
    scenario_id: &str,
    op: &Op,
) -> Result<(), BackendError> {
    let goal_id = op
        .goal_id
        .as_deref()
        .ok_or_else(|| BackendError::usage("attempt_terminal_override missing goal_id"))?;

    let attempt = st.terminal_attempts.entry(goal_id.to_string()).or_insert(0);
    *attempt += 1;

    if st.terminal.contains_key(goal_id) {
        w.emit(json!({
            "type": "terminal_set_attempt",
            "scenario_id": scenario_id,
            "goal_id": goal_id,
            "attempt": *attempt,
            "allowed": false,
            "reason": "terminal_immutable"
        }))?;
        return Ok(());
    }

    w.emit(json!({
        "type": "terminal_set_attempt",
        "scenario_id": scenario_id,
        "goal_id": goal_id,
        "attempt": *attempt,
        "allowed": true
    }))?;

    Ok(())
}

fn exec_wait(op: &Op) -> Result<(), BackendError> {
    let ms = op
        .ms
        .ok_or_else(|| BackendError::usage("wait missing ms"))?;
    std::thread::sleep(Duration::from_millis(ms));
    Ok(())
}
