use crate::error::CoreError;
use crate::model::{Outcome, Scenario, ScenarioBundle};

use anyhow::Result;
use serde_json::{json, Value};
use std::collections::HashMap;

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

    for (scenario_id, scenario) in &bundle.scenarios {
        std::hint::black_box(&scenario.ops);

        let sc_events = events_by_id
            .get(scenario_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);

        let (outcome, details) = evaluate_scenario(scenario, sc_events)?;

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
        // "pass" means: no fails and at least one real pass (not "all skipped").
        "pass": fail_count == 0 && pass_count > 0,
        "summary": { "pass": pass_count, "fail": fail_count, "skip": skip_count },
        "scenarios": scenarios_out
    }))
}

fn evaluate_scenario(scenario: &Scenario, events: &[Value]) -> Result<(Outcome, Value)> {
    let mut detail = serde_json::Map::new();

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
                        CoreError::UnsupportedExpect("custom requires params.must_observe[]".to_string())
                    })?;

                let missing: Vec<_> = must
                    .iter()
                    .filter(|pattern| !events.iter().any(|ev| matches_pattern(ev, pattern)))
                    .collect();

                if !missing.is_empty() {
                    pass = false;
                    detail.insert("missing_expectations".into(), json!(missing));
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
                    ).into());
                }

                let mut found = Vec::new();
                for ev in events.iter().filter(|ev| matches_pattern(ev, where_pat)) {
                    let ty = ev.get("type").and_then(|t| t.as_str()).unwrap_or("");
                    if forbidden.iter().any(|f| *f == ty) {
                        found.push(ev);
                    }
                }

                if !found.is_empty() {
                    pass = false;
                    detail.insert("absent_violation".into(), json!({
                        "where": where_pat,
                        "types": forbidden,
                        "found": found
                    }));
                }
            }

            other => return Err(CoreError::UnsupportedExpect(other.to_string()).into()),
        }
    }

    Ok((if pass { Outcome::Pass } else { Outcome::Fail }, json!(detail)))
}

/// Returns true if `pattern` is a subset of `event`.
fn matches_pattern(event: &Value, pattern: &Value) -> bool {
    let (eobj, pobj) = match (event.as_object(), pattern.as_object()) {
        (Some(e), Some(p)) => (e, p),
        _ => return false,
    };

    pobj.iter().all(|(k, pv)| eobj.get(k) == Some(pv))
}

#[cfg(test)]
mod tests {
    use super::{evaluate_scenario, matches_pattern};
    use crate::model::{Expect, Outcome, Scenario};
    use serde_json::json;

    fn scenario_with_expects(expects: Vec<Expect>) -> Scenario {
        Scenario {
            spec_id: "T00".to_string(),
            title: "test".to_string(),
            layer: None,
            notes: None,
            ops: vec![],
            expects,
        }
    }

    #[test]
    fn matches_pattern_subset_true() {
        let ev = json!({"type":"goal_response","goal_id":"g1","accepted":true,"extra":123});
        let pat = json!({"type":"goal_response","goal_id":"g1","accepted":true});
        assert!(matches_pattern(&ev, &pat));
    }

    #[test]
    fn matches_pattern_subset_false_value_mismatch() {
        let ev = json!({"type":"goal_response","goal_id":"g1","accepted":true});
        let pat = json!({"type":"goal_response","goal_id":"g1","accepted":false});
        assert!(!matches_pattern(&ev, &pat));
    }

    #[test]
    fn scenario_fails_if_missing_start_or_end() {
        let s = scenario_with_expects(vec![Expect {
            check: "custom".to_string(),
            params: Some(json!({"must_observe":[{"type":"x"}]})),
        }]);

        let events = vec![
            json!({"type":"scenario_start"}),
            // no scenario_end
        ];

        let (outcome, _details) = evaluate_scenario(&s, &events).unwrap();
        assert_eq!(outcome, Outcome::Fail);
    }

    #[test]
    fn scenario_skips_if_no_expects_even_if_it_ran() {
        let s = scenario_with_expects(vec![]);
        let events = vec![json!({"type":"scenario_start"}), json!({"type":"scenario_end"})];

        let (outcome, _details) = evaluate_scenario(&s, &events).unwrap();
        assert_eq!(outcome, Outcome::Skip);
    }

    #[test]
    fn custom_passes_when_all_patterns_observed() {
        let s = scenario_with_expects(vec![Expect {
            check: "custom".to_string(),
            params: Some(json!({
                "must_observe": [
                    {"type":"goal_send","goal_id":"g1"},
                    {"type":"goal_response","goal_id":"g1","accepted":true}
                ]
            })),
        }]);

        let events = vec![
            json!({"type":"scenario_start"}),
            json!({"type":"goal_send","goal_id":"g1"}),
            json!({"type":"goal_response","goal_id":"g1","accepted":true}),
            json!({"type":"scenario_end"}),
        ];

        let (outcome, _details) = evaluate_scenario(&s, &events).unwrap();
        assert_eq!(outcome, Outcome::Pass);
    }

    #[test]
    fn absent_fails_when_forbidden_type_present_for_matching_where() {
        let s = scenario_with_expects(vec![Expect {
            check: "absent".to_string(),
            params: Some(json!({
                "where": {"goal_id":"g_reject"},
                "types": ["goal_send","goal_response"]
            })),
        }]);

        let events = vec![
            json!({"type":"scenario_start"}),
            json!({"type":"goal_send","goal_id":"g_reject"}),
            json!({"type":"scenario_end"}),
        ];

        let (outcome, details) = evaluate_scenario(&s, &events).unwrap();
        assert_eq!(outcome, Outcome::Fail);
        assert!(details.get("absent_violation").is_some());
    }

    #[test]
    fn absent_passes_when_forbidden_types_not_present() {
        let s = scenario_with_expects(vec![Expect {
            check: "absent".to_string(),
            params: Some(json!({
                "where": {"goal_id":"g_reject"},
                "types": ["goal_send","goal_response"]
            })),
        }]);

        let events = vec![
            json!({"type":"scenario_start"}),
            // different goal_id -> should not match `where`
            json!({"type":"goal_send","goal_id":"g_other"}),
            json!({"type":"scenario_end"}),
        ];

        let (outcome, _details) = evaluate_scenario(&s, &events).unwrap();
        assert_eq!(outcome, Outcome::Pass);
    }
}
