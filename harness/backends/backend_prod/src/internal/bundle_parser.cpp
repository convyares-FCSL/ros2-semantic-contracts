/// @file bundle_parser.cpp
/// @brief Bundle parsing and validation implementation.
#include "internal/bundle_parser.hpp"
#include "internal/errors.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

namespace backend_prod {

using json = nlohmann::json;

namespace {

Op parse_op(const json& j) {
    Op op;
    
    if (!j.contains("op")) {
        throw BundleError("op missing required 'op' field");
    }
    if (!j["op"].is_string()) {
        throw BundleError("op['op'] must be string");
    }
    op.op = j["op"].get<std::string>();
    
    if (j.contains("goal_id")) {
        if (!j["goal_id"].is_string()) {
            throw BundleError("op['goal_id'] must be string");
        }
        op.goal_id = j["goal_id"].get<std::string>();
    }
    
    if (j.contains("ms")) {
        if (!j["ms"].is_number_unsigned()) {
            throw BundleError("op['ms'] must be unsigned integer");
        }
        op.ms = j["ms"].get<uint64_t>();
    }
    
    if (j.contains("status")) {
        if (!j["status"].is_string()) {
            throw BundleError("op['status'] must be string");
        }
        op.status = j["status"].get<std::string>();
    }

    if (j.contains("actor")) {
        if (!j["actor"].is_string()) {
            throw BundleError("op['actor'] must be string");
        }
        op.actor = j["actor"].get<std::string>();
    }

    if (j.contains("params")) {
        if (!j["params"].is_object()) {
            throw BundleError("op['params'] must be object");
        }
        op.params = j["params"];
    }
    
    if (j.contains("payload")) {
        op.payload = j["payload"];
    }
    
    return op;
}

} // anonymous namespace

Bundle parse_bundle(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw SystemError("cannot open bundle file: " + path);
    }
    
    json root;
    try {
        root = json::parse(ifs);
    } catch (const json::exception& e) {
        throw BundleError("malformed JSON in bundle: " + std::string(e.what()));
    }
    
    // Validate root structure
    if (!root.is_object()) {
        throw BundleError("bundle root must be object");
    }
    
    if (!root.contains("scenarios")) {
        throw BundleError("bundle missing required 'scenarios' key");
    }
    
    if (!root["scenarios"].is_object()) {
        throw BundleError("bundle['scenarios'] must be object, got " + 
                         std::string(root["scenarios"].type_name()));
    }
    
    // Parse scenarios
    Bundle bundle;
    for (auto& [key, val] : root["scenarios"].items()) {
        if (!val.is_object()) {
            throw BundleError("scenario '" + key + "' must be object");
        }
        
        Scenario sc;
        
        if (val.contains("requires")) {
            if (!val["requires"].is_array()) {
                throw BundleError("scenario '" + key + "' field 'requires' must be array");
            }
            sc.required_caps = val["requires"].get<std::vector<std::string>>();
        }
        
        if (!val.contains("ops")) {
            throw BundleError("scenario '" + key + "' missing required 'ops' field");
        }
        
        if (!val["ops"].is_array()) {
            throw BundleError("scenario '" + key + "' field 'ops' must be array");
        }
        
        for (const auto& op_j : val["ops"]) {
            sc.ops.push_back(parse_op(op_j));
        }
        
        bundle.scenarios[key] = sc;
    }
    
    return bundle;
}

} // namespace backend_prod
