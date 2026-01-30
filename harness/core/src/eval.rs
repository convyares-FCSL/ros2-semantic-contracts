use crate::error::CoreError;
use crate::model::{Outcome, Scenario, ScenarioBundle};

use anyhow::Result;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};

pub fn evaluate_bundle(bundle: &ScenarioBundle, events: Vec<Value>) -> Result<Value> {
    let mut events_by_id: HashMap<String, Vec<Value>> = HashMap::new();
    for event in events {
        if let Some(sid) = event.get("scenario_id").and_then(|s| s.as_str()) {
            events_by_id.entry(sid.to_string()).or_default().push(event);
        }
    }

    let mut scenarios_out = serde_json::Map::new();

    let mut pass_count = 0usize;
    let mut fail_count = 0usize;
    let mut skip_count = 0usize;

    // Extract backend capabilities from the _run scope.
    let backend_caps: HashSet<String> = if let Some(run_events) = events_by_id.get("_run") {
        run_events
            .iter()
            .filter(|e| e["type"] == "backend_capabilities")
            .flat_map(|e| {
                e.get("detail")
                    .and_then(|d| d.get("caps"))
                    .and_then(|c| c.as_array())
            })
            .flatten()
            .filter_map(|v| v.as_str())
            .map(|s| s.to_string())
            .collect()
    } else {
        HashSet::new()
    };

    for (scenario_id, scenario) in &bundle.scenarios {
        std::hint::black_box(&scenario.ops);

        let sc_events = events_by_id
            .get(scenario_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);

        let (outcome, details) = evaluate_scenario(scenario, sc_events, &backend_caps)?;

        match outcome {
            Outcome::Pass => pass_count += 1,
            Outcome::Fail => fail_count += 1,
            Outcome::Skip => skip_count += 1,
        }

        scenarios_out.insert(
            scenario_id.clone(),
            json!({
                "outcome": outcome.as_str(),
                "details": details,
                "spec_id": scenario.spec_id,
                "title": scenario.title,
                "layer": scenario.layer,
                "notes": scenario.notes
            }),
        );
    }

    Ok(json!({
        "version": bundle.version,
        "group": bundle.group,
        "description": bundle.description,
        "pass": fail_count == 0 && pass_count > 0,
        "summary": { "pass": pass_count, "fail": fail_count, "skip": skip_count },
        "scenarios": scenarios_out
    }))
}

fn evaluate_scenario(
    scenario: &Scenario,
    events: &[Value],
    caps: &HashSet<String>,
) -> Result<(Outcome, Value)> {
    let mut detail = serde_json::Map::new();
    let mut violations = serde_json::Map::new();

    // Capability gating
    if !scenario.requires.is_empty() {
        let missing: Vec<_> = scenario
            .requires
            .iter()
            .filter(|req| !caps.contains(*req))
            .cloned()
            .collect();

        if !missing.is_empty() {
            detail.insert("capability_missing".into(), json!(missing));
            return Ok((Outcome::Skip, json!(detail)));
        }
    }

    let has_start = events.iter().any(|e| e["type"] == "scenario_start");
    let has_end = events.iter().any(|e| e["type"] == "scenario_end");

    detail.insert("observed_start".into(), json!(has_start));
    detail.insert("observed_end".into(), json!(has_end));

    if !has_start || !has_end {
        return Ok((Outcome::Fail, json!(detail)));
    }

    if scenario.expects.is_empty() {
        return Ok((Outcome::Skip, json!(detail)));
    }

    let mut pass = true;

    for ex in &scenario.expects {
        match ex.check.as_str() {
            "custom" => {
                let params = ex.params.as_ref().unwrap_or(&Value::Null);
                let must = params
                    .get("must_observe")
                    .and_then(|v| v.as_array())
                    .ok_or_else(|| {
                        CoreError::UnsupportedExpect(
                            "custom requires params.must_observe[]".to_string(),
                        )
                    })?;

                let missing: Vec<_> = must
                    .iter()
                    .filter(|pattern| !events.iter().any(|ev| matches_pattern(ev, pattern)))
                    .cloned()
                    .collect();

                if !missing.is_empty() {
                    pass = false;
                    violations
                        .entry("missing_expectations")
                        .or_insert_with(|| json!([]))
                        .as_array_mut()
                        .unwrap()
                        .extend(missing);
                }
            }

            "absent" => {
                let params = ex.params.as_ref().unwrap_or(&Value::Null);
                let where_pat = params.get("where").ok_or_else(|| {
                    CoreError::UnsupportedExpect("absent requires params.where".to_string())
                })?;

                let types = params
                    .get("types")
                    .and_then(|v| v.as_array())
                    .ok_or_else(|| {
                        CoreError::UnsupportedExpect("absent requires params.types[]".to_string())
                    })?;

                let forbidden: Vec<&str> = types.iter().filter_map(|v| v.as_str()).collect();
                if forbidden.is_empty() {
                    return Err(CoreError::UnsupportedExpect(
                        "absent params.types[] must contain at least one string".to_string(),
                    )
                    .into());
                }

                let mut found = Vec::new();
                for ev in events.iter().filter(|ev| matches_pattern(ev, where_pat)) {
                    let ty = ev.get("type").and_then(|t| t.as_str()).unwrap_or("");
                    if forbidden.iter().any(|f| *f == ty) {
                        found.push(ev.clone());
                    }
                }

                if !found.is_empty() {
                    pass = false;
                    violations
                        .entry("absent_violation")
                        .or_insert_with(|| json!([]))
                        .as_array_mut()
                        .unwrap()
                        .push(json!({
                            "where": where_pat,
                            "types": forbidden,
                            "found": found
                        }));
                }
            }

            // P12: compare a scenario-declared evidence sub-value across repeated responses.
            //
            // This check is intentionally strict:
            // - The scenario declares *what* is compared via params.evidence_ptr (JSON Pointer).
            // - The core does NOT strip headers or guess semantics.
            "describe_unknown_consistent" => {
                let params = ex.params.as_ref().unwrap_or(&Value::Null);

                let name = params.get("name").and_then(|v| v.as_str()).ok_or_else(|| {
                    CoreError::UnsupportedExpect(
                        "describe_unknown_consistent requires params.name".to_string(),
                    )
                })?;

                let min_calls = params
                    .get("min_calls")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(2);
                if min_calls < 2 {
                    return Err(CoreError::UnsupportedExpect(
                        "describe_unknown_consistent params.min_calls must be >= 2".to_string(),
                    )
                    .into());
                }

                // JSON Pointer (RFC 6901), e.g. "/detail"
                let evidence_ptr = params
                    .get("evidence_ptr")
                    .and_then(|v| v.as_str())
                    .ok_or_else(|| {
                        CoreError::UnsupportedExpect(
                            "describe_unknown_consistent requires params.evidence_ptr (JSON Pointer, e.g. \"/detail\")"
                                .to_string(),
                        )
                    })?;

                let responses: Vec<&Value> = events
                    .iter()
                    .filter(|e| {
                        e.get("type").and_then(|v| v.as_str()) == Some("param_describe_response")
                    })
                    .filter(|e| e.get("name").and_then(|v| v.as_str()) == Some(name))
                    .collect();

                if (responses.len() as u64) < min_calls {
                    pass = false;
                    violations
                        .entry("describe_unknown_consistent_violation")
                        .or_insert_with(|| json!([]))
                        .as_array_mut()
                        .unwrap()
                        .push(json!({
                            "name": name,
                            "reason": "insufficient_responses",
                            "observed_calls": responses.len(),
                            "min_calls": min_calls
                        }));
                    continue;
                }

                let mut extracted: Vec<Value> = Vec::with_capacity(responses.len());
                let mut missing_count = 0usize;

                for ev in &responses {
                    match ev.pointer(evidence_ptr) {
                        Some(v) => extracted.push(v.clone()),
                        None => missing_count += 1,
                    }
                }

                if missing_count > 0 {
                    pass = false;
                    violations
                        .entry("describe_unknown_consistent_violation")
                        .or_insert_with(|| json!([]))
                        .as_array_mut()
                        .unwrap()
                        .push(json!({
                            "name": name,
                            "reason": "missing_evidence",
                            "evidence_ptr": evidence_ptr,
                            "missing_count": missing_count
                        }));
                    continue;
                }

                let first = &extracted[0];
                if !extracted.iter().all(|v| v == first) {
                    pass = false;
                    violations
                        .entry("describe_unknown_consistent_violation")
                        .or_insert_with(|| json!([]))
                        .as_array_mut()
                        .unwrap()
                        .push(json!({
                            "name": name,
                            "reason": "evidence_differs",
                            "evidence_ptr": evidence_ptr,
                            "evidence": extracted
                        }));
                }
            }

            other => return Err(CoreError::UnsupportedExpect(other.to_string()).into()),
        }
    }

    if !violations.is_empty() {
        detail.insert("violations".into(), json!(violations));
    }

    Ok((
        if pass { Outcome::Pass } else { Outcome::Fail },
        json!(detail),
    ))
}

/// Recursive subset matcher used by custom expectations.
///
/// Semantics:
/// - Objects: every key/value in `pattern` must exist in `event` and match recursively.
/// - Arrays: exact equality (intentional for now; subset semantics can be added later).
/// - Scalars: equality.
///
/// This allows expectations to match structured evidence without requiring full
/// event equality, while remaining deterministic and explicit.
fn matches_pattern(event: &Value, pattern: &Value) -> bool {
    match (event, pattern) {
        (Value::Object(e), Value::Object(p)) => p.iter().all(|(k, pv)| match e.get(k) {
            Some(ev) => matches_pattern(ev, pv),
            None => false,
        }),
        (Value::Array(ea), Value::Array(pa)) => ea == pa,
        _ => event == pattern,
    }
}
