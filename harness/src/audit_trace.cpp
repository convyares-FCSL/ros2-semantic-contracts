#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

static constexpr const char* GREEN = "\033[32m";
static constexpr const char* RED   = "\033[31m";
static constexpr const char* BOLD  = "\033[1m";
static constexpr const char* RESET = "\033[0m";

struct Status {
  bool seen_end{false};
  bool ok{true};
};

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: audit_trace <trace.jsonl>\n";
    return 2;
  }

  std::ifstream in(argv[1]);
  if (!in) {
    std::cerr << "failed to open trace file\n";
    return 2;
  }

  // Run metadata (from trace; auditable)
  std::string backend = "unknown";
  std::string ros_distro;
  std::string rmw_impl;

  std::unordered_map<std::string, Status> scenarios;
  std::string line;

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    json ev = json::parse(line);

    const std::string type = ev.value("type", "");
    const std::string scenario_id = ev.value("scenario_id", "");

    if (type == "run_start") {
      if (ev.contains("detail") && ev["detail"].is_object()) {
        const auto& d = ev["detail"];
        backend = d.value("backend", backend);
        ros_distro = d.value("ros_distro", "");
        rmw_impl = d.value("rmw_implementation", "");
      }
      continue;
    }

    if (scenario_id.empty()) continue;

    if (type == "assertion") {
      if (ev.contains("detail") && ev["detail"].is_object() && ev["detail"].contains("ok")) {
        bool ok = ev["detail"]["ok"].get<bool>();
        scenarios[scenario_id].ok = scenarios[scenario_id].ok && ok;
      }
    } else if (type == "scenario_end") {
      scenarios[scenario_id].seen_end = true;
      if (ev.contains("detail") && ev["detail"].is_object() && ev["detail"].contains("ok")) {
        scenarios[scenario_id].ok = scenarios[scenario_id].ok && ev["detail"]["ok"].get<bool>();
      }
    }
  }

  // Print what was tested (single, clear line)
  std::cout << "Oracle backend: " << backend;
  if (!ros_distro.empty() || !rmw_impl.empty()) {
    std::cout << " (";
    bool first = true;
    if (!ros_distro.empty()) {
      std::cout << "ROS " << ros_distro;
      first = false;
    }
    if (!rmw_impl.empty()) {
      if (!first) std::cout << ", ";
      std::cout << rmw_impl;
    }
    std::cout << ")";
  }
  std::cout << "\n\n";

  // Stable output order
  std::vector<std::string> ids;
  ids.reserve(scenarios.size());
  for (const auto& [id, _] : scenarios) ids.push_back(id);
  std::sort(ids.begin(), ids.end());

  bool all_ok = true;

  for (const auto& id : ids) {
    const auto& st = scenarios[id];

    if (!st.seen_end) {
      all_ok = false;
      std::cout << id << " : " << RED << "FAILED" << RESET << ". (missing scenario_end)\n";
      continue;
    }

    if (st.ok) {
      std::cout << id << " : " << GREEN << "PASSED" << RESET << ".\n";
    } else {
      all_ok = false;
      std::cout << id << " : " << RED << "FAILED" << RESET << ".\n";
    }
  }

  if (all_ok) {
    std::cout << BOLD << GREEN << "ALL SPEC PASSED." << RESET << "\n";
    return 0;
  } else {
    std::cout << BOLD << RED << "SOME SPEC FAILED." << RESET << "\n";
    return 1;
  }
}
