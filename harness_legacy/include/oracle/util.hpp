#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

namespace oracle::util {

using json = nlohmann::json;

inline uint64_t now_ns() {
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
  return static_cast<uint64_t>(ns);
}

inline std::string getenv_or(const char* key, const std::string& fallback) {
  const char* v = std::getenv(key);
  if (!v || std::string(v).empty()) return fallback;
  return std::string(v);
}

inline json base_event(const std::string& version,
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

inline void write_event(std::ofstream& out, const json& ev) { out << ev.dump() << "\n"; }

inline bool json_has_string(const json& j, const std::string& key) {
  return j.contains(key) && j[key].is_string() && !j[key].get<std::string>().empty();
}

}  // namespace oracle::util
