use crate::{
    bundle::{Op, Scenario},
    error::BackendError,
    io::EventWriter,
    state::BackendState,
};
use serde_json::json;
use std::time::Duration;

use rclrs::vendor::rcl_interfaces::{msg::ParameterValue, srv::SetParameters};

pub fn run_scenario(
    w: &mut EventWriter,
    id: &str,
    scenario: &Scenario,
    param_client: &rclrs::Client<SetParameters>,
) -> Result<(), BackendError> {
    let mut st = BackendState::default();

    w.emit(json!({
        "type": "scenario_start",
        "scenario_id": id,
        "detail": { "backend": "ros" }
    }))?;

    for (i, op) in scenario.ops.iter().enumerate() {
        let op_id = format!("{id}#{i}");
        exec_op(w, &mut st, id, &op_id, op, param_client)?;
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
    param_client: &rclrs::Client<SetParameters>,
) -> Result<(), BackendError> {
    w.emit(json!({
        "type": "op_start",
        "scenario_id": scenario_id,
        "op_id": op_id,
        "detail": { "op": op.op }
    }))?;

    match op.op.as_str() {
        "send_goal" => exec_send_goal(w, scenario_id, op)?,
        "wait" => exec_wait(op)?,
        "complete_terminal" => exec_complete_terminal(w, st, scenario_id, op)?,
        "attempt_terminal_override" => exec_attempt_terminal_override(w, st, scenario_id, op)?,

        "set_param" => exec_set_param(w, scenario_id, op, param_client)?,

        other => {
            w.emit(json!({
                "type": "diagnostic",
                "scenario_id": scenario_id,
                "detail": { "error": { "kind": "unsupported_op", "op": other } }
            }))?;
            return Err(BackendError::usage(format!("unsupported op: {other}")));
        }
    }

    w.emit(json!({
        "type": "op_end",
        "scenario_id": scenario_id,
        "op_id": op_id
    }))?;

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
        .unwrap_or("string"); // default/fallback if needed, though schema should enforce

    // Map to ParameterValue
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
        } // Arrays valid but skipping for P06 scope
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

    // Block on response (spinner thread is running)
    let result = futures::executor::block_on(future);

    match result {
        Ok((response, _info)) => {
            // response.results is Vec<SetParametersResult>
            let result = response
                .results
                .first()
                .ok_or_else(|| BackendError::system(anyhow::anyhow!("empty response results")))?;

            w.emit(json!({
                "type": "param_set_response",
                "scenario_id": scenario_id,
                "node": "/oracle_backend_ros",
                "name": name,
                "successful": result.successful,
                "reason": result.reason
            }))?;
        }
        Err(e) => {
            // Call failed/canceled
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
