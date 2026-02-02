/// @file types.hpp
/// @brief Internal type definitions for bundle parsing.
#ifndef BACKEND_PROD_TYPES_HPP_
#define BACKEND_PROD_TYPES_HPP_

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <cstdint>
#include <nlohmann/json.hpp>

namespace backend_prod {

using json = nlohmann::json;

/// Operation parsed from scenario bundle.
struct Op {
    std::string op;
    std::optional<std::string> goal_id;
    std::optional<uint64_t> ms;
    std::optional<std::string> status;
    std::optional<json> payload;
};

/// Scenario from bundle.
struct Scenario {
    std::vector<std::string> required_caps;
    std::vector<Op> ops;
};

/// Top-level bundle.
struct Bundle {
    std::map<std::string, Scenario> scenarios;
};

} // namespace backend_prod

#endif // BACKEND_PROD_TYPES_HPP_
