use serde_json::Value;
use std::io::IsTerminal;

pub struct SummaryCounts {
    pub pass: usize,
    pub fail: usize,
    pub skip: usize,
}

pub fn print_summary(report: &Value) -> SummaryCounts {
    let scenarios = match report.get("scenarios").and_then(|v| v.as_object()) {
        Some(s) => s,
        None => {
            println!("No scenarios in report.");
            return SummaryCounts { pass: 0, fail: 0, skip: 0 };
        }
    };

    let mut ids: Vec<&String> = scenarios.keys().collect();
    ids.sort();

    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;

    for id in ids {
        let sc = &scenarios[id];
        let outcome = sc.get("outcome").and_then(|v| v.as_str()).unwrap_or("FAIL");
        let title = sc.get("title").and_then(|v| v.as_str()).unwrap_or("");
        let spec_id = sc.get("spec_id").and_then(|v| v.as_str()).unwrap_or("");

        let label = if title.is_empty() {
            format!("{id} ({spec_id})")
        } else {
            format!("{id} ({spec_id}) — {title}")
        };

        match outcome {
            "PASS" => { pass += 1; println!("{} {}", green("PASS"), label); }
            "SKIP" => { skip += 1; println!("{} {}", yellow("SKIP"), label); }
            _ => { fail += 1; println!("{} {}", red("FAIL"), label); }
        }
    }

    println!();

    match (fail, pass, skip) {
        (0, 0, s) if s > 0 => println!("{}", yellow(&format!("ALL TESTS SKIPPED ({s} skipped)"))),
        (0, _, s) => {
            if s == 0 { println!("{}", green("ALL TESTS PASSED")); }
            else { println!("{}", green(&format!("ALL TESTS PASSED ({s} skipped)"))); }
        }
        (f, _, s) => {
            if s == 0 { println!("{}", red(&format!("{f} TEST(S) FAILED"))); }
            else { println!("{}", red(&format!("{f} TEST(S) FAILED ({s} skipped)"))); }
        }
    }

    SummaryCounts { pass, fail, skip }
}

fn green(s: &str) -> String { colorize(s, "32") }
fn yellow(s: &str) -> String { colorize(s, "33") }
fn red(s: &str) -> String { colorize(s, "31") }

fn colorize(s: &str, code: &str) -> String {
    if std::io::stdout().is_terminal() {
        format!("\x1b[{}m{}\x1b[0m", code, s)
    } else {
        s.to_string()
    }
}
