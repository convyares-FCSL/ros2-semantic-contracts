#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

static uint64_t now_ns() {
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
  return static_cast<uint64_t>(ns);
}

static void write_event(std::ofstream& out, const json& ev) { out << ev.dump() << "\n"; }

static json base_event(const std::string& version,
                       const std::string& run_id,
                       const std::string& type,
                       const std::string& scenario_id) {
  json ev;
  ev["version"] = version;
  ev["run_id"] = run_id;
  ev["t_ns"] = now_ns();
  ev["type"] = type;
  ev["scenario_id"] = scenario_id;
  return ev;
}

static json load_json_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open JSON file: " + path);
  json j;
  in >> j;
  return j;
}

static bool json_has_string(const json& j, const std::string& key) {
  return j.contains(key) && j[key].is_string() && !j[key].get<std::string>().empty();
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: oracle_a <scenarios.json> <trace.jsonl>\n";
    return 2;
  }

  const std::string scenarios_path = argv[1];
  const std::string trace_path = argv[2];

  std::ofstream trace(trace_path, std::ios::out | std::ios::trunc);
  if (!trace) {
    std::cerr << "failed to open trace file: " << trace_path << "\n";
    return 2;
  }

  // Keep stable for now; can become CLI args later.
  const std::string version = "0.1";
  const std::string run_id = "dev";

  json root;
  try {
    root = load_json_file(scenarios_path);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }

  if (!root.contains("scenarios") || !root["scenarios"].is_array()) {
    std::cerr << "invalid scenarios file: missing 'scenarios' array\n";
    return 2;
  }

  // Run each scenario; emit JSONL events only. No console summary.
  for (const auto& sc : root["scenarios"]) {
    if (!json_has_string(sc, "id")) {
      std::cerr << "scenario missing id\n";
      return 2;
    }
    const std::string scenario_id = sc["id"].get<std::string>();

    // spec_id optional. If absent, fall back to scenario id (used only for trace context).
    const std::string spec_id = json_has_string(sc, "spec_id") ? sc["spec_id"].get<std::string>()
                                                               : scenario_id;

    // scenario_start
    {
      auto ev = base_event(version, run_id, "scenario_start", scenario_id);
      ev["detail"] = json::object({{"source", scenarios_path}, {"spec_id", spec_id}});
      write_event(trace, ev);
    }

    // Validate steps
    if (!sc.contains("steps") || !sc["steps"].is_array() || sc["steps"].empty()) {
      auto ev = base_event(version, run_id, "scenario_end", scenario_id);
      ev["detail"] =
          json::object({{"ok", false}, {"reason", "missing or empty steps array"}, {"spec_id", spec_id}});
      write_event(trace, ev);
      continue;
    }

    // --- Execution state (STUB MODEL for A, pre-ROS) ---
    bool accepted_one = false;
    std::unordered_set<std::string> accepted_goals;

    std::unordered_map<std::string, json> last_goal_send_decision;
    std::unordered_map<std::string, json> last_cancel_response;

    bool scenario_ok = true;
    std::string scenario_reason;

    auto fail = [&](const std::string& why) {
      scenario_ok = false;
      if (scenario_reason.empty()) scenario_reason = why;
    };

    auto emit_assert_event = [&](bool ok, const json& expected, const json& observed) {
      auto as = base_event(version, run_id, "assertion", scenario_id);
      as["detail"] = json::object({
          {"ok", ok},
          {"expected", expected},
          {"observed", observed},
          {"spec_id", spec_id},
      });
      write_event(trace, as);
      if (!ok) fail("assertion failed");
    };

    for (const auto& step : sc["steps"]) {
      if (!step.contains("op") || !step["op"].is_string()) {
        emit_assert_event(false, step, json::object({{"note", "step missing op"}}));
        break;
      }
      const std::string op = step["op"].get<std::string>();

      if (op == "send_goal") {
        if (!json_has_string(step, "goal_id")) {
          emit_assert_event(false, step, json::object({{"note", "send_goal missing goal_id"}}));
          break;
        }
        const std::string goal_id = step["goal_id"].get<std::string>();

        bool accepted = false;
        std::string reason;

        if (!accepted_one) {
          accepted = true;
          accepted_one = true;
          accepted_goals.insert(goal_id);
        } else {
          accepted = false;
          reason = "ConcurrencyPolicy";  // stubbed for now
        }

        auto ev = base_event(version, run_id, "goal_send_decision", scenario_id);
        ev["goal_id"] = goal_id;
        ev["accepted"] = accepted;
        if (!accepted) ev["reason"] = reason;
        ev["detail"] = json::object({{"spec_id", spec_id}});
        write_event(trace, ev);

        last_goal_send_decision[goal_id] = ev;
      } else if (op == "request_cancel") {
        if (!json_has_string(step, "goal_id")) {
          emit_assert_event(false, step, json::object({{"note", "request_cancel missing goal_id"}}));
          break;
        }
        const std::string goal_id = step["goal_id"].get<std::string>();

        {
          auto req = base_event(version, run_id, "cancel_request", scenario_id);
          req["goal_id"] = goal_id;
          req["detail"] = json::object({{"spec_id", spec_id}});
          write_event(trace, req);
        }

        const bool accepted = accepted_goals.count(goal_id) > 0;
        const std::string reason = accepted ? "" : "UnknownGoal";

        auto resp = base_event(version, run_id, "cancel_response", scenario_id);
        resp["goal_id"] = goal_id;
        resp["accepted"] = accepted;
        if (!accepted) resp["reason"] = reason;
        resp["detail"] = json::object({{"spec_id", spec_id}});
        write_event(trace, resp);

        last_cancel_response[goal_id] = resp;
      } else if (op == "assert") {
        const json expected = step.value("assert", json::object());
        const std::string atype = expected.value("type", "");

        if (atype == "goal_send_decision") {
          const std::string goal_id = expected.value("goal_id", "");
          if (goal_id.empty()) {
            emit_assert_event(false, expected, json::object({{"note", "assert missing goal_id"}}));
            continue;
          }
          auto it = last_goal_send_decision.find(goal_id);
          if (it == last_goal_send_decision.end()) {
            emit_assert_event(false, expected, json::object({{"note", "no prior goal_send_decision"}}));
            continue;
          }

          const json& obs = it->second;
          bool ok = true;

          if (expected.contains("accepted")) {
            if (obs.value("accepted", false) != expected["accepted"].get<bool>()) ok = false;
          }
          if (expected.contains("reason")) {
            if (obs.value("reason", "") != expected["reason"].get<std::string>()) ok = false;
          }

          emit_assert_event(ok, expected, obs);
        } else if (atype == "cancel_response") {
          const std::string goal_id = expected.value("goal_id", "");
          if (goal_id.empty()) {
            emit_assert_event(false, expected, json::object({{"note", "assert missing goal_id"}}));
            continue;
          }
          auto it = last_cancel_response.find(goal_id);
          if (it == last_cancel_response.end()) {
            emit_assert_event(false, expected, json::object({{"note", "no prior cancel_response"}}));
            continue;
          }

          const json& obs = it->second;
          bool ok = true;

          if (expected.contains("accepted")) {
            if (obs.value("accepted", false) != expected["accepted"].get<bool>()) ok = false;
          }
          if (expected.contains("reason")) {
            if (obs.value("reason", "") != expected["reason"].get<std::string>()) ok = false;
          }

          emit_assert_event(ok, expected, obs);
        } else {
          emit_assert_event(false, expected,
                            json::object({{"note", "unsupported assert.type"}, {"type", atype}}));
        }
      } else if (op == "wait") {
        auto ev = base_event(version, run_id, "assertion", scenario_id);
        ev["detail"] = json::object({
            {"ok", true},
            {"note", "wait is currently record-only (no ROS wired yet)"},
            {"ms", step.value("ms", 0)},
            {"spec_id", spec_id},
        });
        write_event(trace, ev);
      } else if (op == "emit_feedback") {
        auto ev = base_event(version, run_id, "assertion", scenario_id);
        ev["detail"] = json::object({
            {"ok", true},
            {"note", "emit_feedback is currently record-only (no ROS wired yet)"},
            {"goal_id", step.value("goal_id", "")},
            {"spec_id", spec_id},
        });
        write_event(trace, ev);
      } else {
        emit_assert_event(false, step, json::object({{"note", "unknown op"}, {"op", op}}));
      }

      if (!scenario_ok) break;
    }

    // scenario_end
    {
      auto ev = base_event(version, run_id, "scenario_end", scenario_id);
      ev["detail"] = json::object({{"ok", scenario_ok}, {"spec_id", spec_id}});
      if (!scenario_ok && !scenario_reason.empty()) ev["detail"]["reason"] = scenario_reason;
      write_event(trace, ev);
    }
  }

  // Trace is the source of truth; summary is done by audit_trace.
  return 0;
}
