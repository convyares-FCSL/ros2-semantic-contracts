#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace oracle {

class TraceWriter {
 public:
  using json = nlohmann::json;

  explicit TraceWriter(std::ofstream& out) : out_(out) {}

  void write(const json& ev) {
    std::lock_guard<std::mutex> lock(mu_);
    out_ << ev.dump() << "\n";

    const std::string type = ev.value("type", "");
    const std::string goal_id = ev.value("goal_id", "");
    if (!type.empty() && !goal_id.empty()) {
      last_by_type_goal_[type + ":" + goal_id] = ev;
    }
    if (!type.empty()) {
      last_by_type_[type] = ev;
    }
  }

  json last_for(const std::string& type, const std::string& goal_id) const {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = last_by_type_goal_.find(type + ":" + goal_id);
    if (it == last_by_type_goal_.end()) return json();
    return it->second;
  }

  json last_for(const std::string& type) const {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = last_by_type_.find(type);
    if (it == last_by_type_.end()) return json();
    return it->second;
  }

 private:
  std::ofstream& out_;
  mutable std::mutex mu_;
  std::unordered_map<std::string, json> last_by_type_goal_;
  std::unordered_map<std::string, json> last_by_type_;
};

}  // namespace oracle
