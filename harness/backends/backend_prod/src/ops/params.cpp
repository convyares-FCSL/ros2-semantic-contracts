#include "../backend.hpp"
#include "internal/errors.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <set>
#include <thread>

namespace backend_prod::ops {

// payload contract: "name" required (checked by caller), "value" required (checked by caller).
// "type" defaults to "string" if absent.
static rclcpp::Parameter make_parameter(const std::string& name, const json& payload) {
    std::string type = payload.contains("type") ? payload["type"].get<std::string>() : "string";
    if (type == "integer")  return rclcpp::Parameter(name, payload["value"].get<int64_t>());
    if (type == "double")   return rclcpp::Parameter(name, payload["value"].get<double>());
    if (type == "string")   return rclcpp::Parameter(name, payload["value"].get<std::string>());
    if (type == "bool")     return rclcpp::Parameter(name, payload["value"].get<bool>());
    throw BundleError("set_param: unsupported type '" + type + "'");
}

static rclcpp::Parameter make_parameter_from_value(
    const std::string& name, const json& value
) {
    if (value.is_boolean()) {
        return rclcpp::Parameter(name, value.get<bool>());
    }
    if (value.is_number_integer()) {
        return rclcpp::Parameter(name, value.get<int64_t>());
    }
    if (value.is_number_float()) {
        return rclcpp::Parameter(name, value.get<double>());
    }
    if (value.is_string()) {
        return rclcpp::Parameter(name, value.get<std::string>());
    }
    throw BundleError("declare_param: unsupported value type for '" + name + "'");
}

static std::string parameter_to_string(const rclcpp::Parameter& param) {
    switch (param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return param.as_bool() ? "true" : "false";
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return std::to_string(param.as_int());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return std::to_string(param.as_double());
        case rclcpp::ParameterType::PARAMETER_STRING:
            return param.as_string();
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
            return "NOT_SET";
        default:
            return "UNSUPPORTED";
    }
}

static std::vector<rclcpp::Parameter> build_parameter_list(
    const json& params_array, const std::string& op_name
) {
    if (!params_array.is_array()) {
        throw BundleError(op_name + ": 'params' must be an array");
    }

    std::vector<rclcpp::Parameter> params;
    for (const auto& p : params_array) {
        if (!p.count("name") || !p.count("value")) {
            throw BundleError(op_name + ": each param must have 'name' and 'value'");
        }
        params.push_back(make_parameter(p["name"], p));
    }
    return params;
}

static json parameter_value_to_json(const rcl_interfaces::msg::ParameterValue& value) {
    json out = {
        {"type", static_cast<int>(value.type)},
    };

    switch (value.type) {
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
            out["value"] = value.bool_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
            out["value"] = value.integer_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
            out["value"] = value.double_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
            out["value"] = value.string_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
            out["value"] = value.byte_array_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
            out["value"] = value.bool_array_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
            out["value"] = value.integer_array_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
            out["value"] = value.double_array_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
            out["value"] = value.string_array_value;
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET:
        default:
            break;
    }

    return out;
}

static json parameter_to_json(const rcl_interfaces::msg::Parameter& param) {
    json out = {
        {"name", param.name},
    };
    json value = parameter_value_to_json(param.value);
    out["type"] = value["type"];
    if (value.contains("value")) {
        out["value"] = value["value"];
    }
    return out;
}

static size_t drain_parameter_event_buffer(TraceWriter& w, BackendState& st) {
    std::deque<json> drained;
    {
        std::lock_guard<std::mutex> lock(st.param_event_buffer_mutex);
        drained.swap(st.pending_param_events);
    }

    for (const auto& ev : drained) {
        w.emit(ev);
    }
    return drained.size();
}

json exec_set_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("name"))
        throw BundleError("set_param: payload missing 'name'");
    if (!op.payload->count("value"))
        throw BundleError("set_param: payload missing 'value'");
    std::string name = (*op.payload)["name"];

    json result;
    result["op"]   = "set_param";
    result["name"] = name;

    try {
        rcl_interfaces::msg::SetParametersResult res = st.node->set_parameter(make_parameter(name, *op.payload));
        result["successful"] = res.successful;
        if (!res.successful && !res.reason.empty())
            result["reason"] = res.reason;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        result["successful"] = false;
        result["reason"]     = "NotDeclared";
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"]     = std::string("Exception: ") + e.what();
    }
    return result;
}

json exec_declare_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("name")) {
        throw BundleError("declare_param: payload missing 'name'");
    }
    std::string name = (*op.payload)["name"];

    json result;
    result["op"] = "declare_param";
    result["name"] = name;

    // Parse descriptor (optional)
    rcl_interfaces::msg::ParameterDescriptor desc;
    if (op.payload->contains("descriptor")) {
        const json& d = (*op.payload)["descriptor"];
        if (d.contains("read_only")) desc.read_only = d["read_only"].get<bool>();
        if (d.contains("dynamic_typing")) desc.dynamic_typing = d["dynamic_typing"].get<bool>();
        if (d.contains("type")) desc.type = static_cast<uint8_t>(d["type"].get<int>());
    }

    try {
        if (op.payload->contains("value")) {
            const json& value = (*op.payload)["value"];
            if (op.payload->contains("type")) {
                rclcpp::Parameter p = make_parameter(name, *op.payload);
                st.node->declare_parameter(name, p.get_parameter_value(), desc);
            } else {
                rclcpp::Parameter p = make_parameter_from_value(name, value);
                st.node->declare_parameter(name, p.get_parameter_value(), desc);
            }
        } else {
            if (desc.type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
                auto requested_type = static_cast<rclcpp::ParameterType>(desc.type);
                st.node->declare_parameter(name, requested_type, desc);
            } else {
                // Preserve legacy harness behavior when no explicit type is provided.
                st.node->declare_parameter(name, std::string(""), desc);
            }
        }
        result["declared"] = true;

        // Echo descriptor for trace inspection
        if (desc.read_only || desc.type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
            result["descriptor"] = json::object();
            if (desc.read_only) {
                result["descriptor"]["read_only"] = true;
            }
            if (desc.type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
                result["descriptor"]["type"] = static_cast<int>(desc.type);
            }
        }
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        result["declared"] = false;
        result["reason"] = "AlreadyDeclared";
    } catch (const std::exception& e) {
        result["declared"] = false;
        result["reason"] = std::string("Exception: ") + e.what();
    }

    return result;
}

json exec_get_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("name"))
        throw BundleError("get_param: payload missing 'name'");

    std::string name = (*op.payload)["name"];
    json result;
    result["op"] = "get_param";
    result["name"] = name;

    // Check declaration status first
    if (!st.node->has_parameter(name)) {
        result["declared"] = false;
        result["set"] = false;
        result["type"] = "NOT_SET";
        return result;
    }

    // It is declared
    result["declared"] = true;
    rclcpp::Parameter param;
    try {
        param = st.node->get_parameter(name);
    } catch (const rclcpp::exceptions::ParameterUninitializedException&) {
        result["set"] = false;
        result["type"] = "NOT_SET";
        return result;
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result["set"] = false;
        result["type"] = "NOT_SET";
        return result;
    }

    // It is set (typed)
    result["set"] = true;
    switch (param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            result["value"] = param.as_int();
            result["type"] = "integer";
            break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            result["value"] = param.as_double();
            result["type"] = "double";
            break;
        case rclcpp::ParameterType::PARAMETER_STRING:
            result["value"] = param.as_string();
            result["type"] = "string";
            break;
        case rclcpp::ParameterType::PARAMETER_BOOL:
            result["value"] = param.as_bool();
            result["type"] = "bool";
            break;
        default:
            // For array types or unknowns that are backed by a value (not NOT_SET)
            // we mark them as set but don't extract the value yet.
            result["type"] = "unknown";
            break;
    }

    return result;
}

json exec_set_params_batch(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("params"))
        throw BundleError("set_params_batch: payload missing 'params'");

    std::vector<rclcpp::Parameter> params = build_parameter_list(
        (*op.payload)["params"], "set_params_batch"
    );

    json result;
    result["op"] = "set_params_batch";
    result["count"] = params.size();

    try {
        rcl_interfaces::msg::SetParametersResult res = st.node->set_parameters_atomically(params);
        result["successful"] = res.successful;
        if (!res.successful && !res.reason.empty())
            result["reason"] = res.reason;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        result["successful"] = false;
        result["reason"] = "NotDeclared";
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = std::string("Exception: ") + e.what();
    }

    return result;
}

json exec_set_params_non_atomic(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload || !op.payload->count("params")) {
        throw BundleError("set_params_non_atomic: payload missing 'params'");
    }

    std::vector<rclcpp::Parameter> params = build_parameter_list(
        (*op.payload)["params"], "set_params_non_atomic"
    );

    json result = {
        {"op", "set_params_non_atomic"},
        {"count", params.size()},
        {"successful", true},
        {"results", json::array()},
    };

    try {
        auto responses = st.node->set_parameters(params);
        json per_param = json::array();
        bool all_successful = true;

        for (size_t i = 0; i < responses.size(); ++i) {
            const auto& r = responses[i];
            json item = {
                {"name", params[i].get_name()},
                {"successful", r.successful},
            };
            if (!r.successful && !r.reason.empty()) {
                item["reason"] = r.reason;
            }
            if (!r.successful) {
                all_successful = false;
            }
            per_param.push_back(item);
        }

        result["successful"] = all_successful;
        result["results"] = per_param;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        result["successful"] = false;
        result["reason"] = "NotDeclared";
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = std::string("Exception: ") + e.what();
    }

    return result;
}

json exec_register_param_callback(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload) {
        throw BundleError("register_param_callback: missing payload");
    }

    const json& payload = *op.payload;
    std::string mode = payload.value("mode", "");
    if (mode != "reject_names") {
        throw BundleError("register_param_callback: unsupported mode '" + mode + "'");
    }
    if (!payload.contains("reject") || !payload["reject"].is_array()) {
        throw BundleError("register_param_callback: payload.reject must be an array");
    }

    std::set<std::string> reject_names;
    for (const auto& item : payload["reject"]) {
        if (!item.is_string()) {
            throw BundleError("register_param_callback: payload.reject[] must be strings");
        }
        reject_names.insert(item.get<std::string>());
    }

    if (st.on_set_params_cb_handle) {
        st.node->remove_on_set_parameters_callback(st.on_set_params_cb_handle.get());
        st.on_set_params_cb_handle.reset();
    }

    st.on_set_params_cb_handle = st.node->add_on_set_parameters_callback(
        [reject_names](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;

            for (const auto& p : params) {
                if (reject_names.count(p.get_name()) > 0) {
                    res.successful = false;
                    res.reason = "RejectedByCallback:" + p.get_name();
                    return res;
                }
            }

            return res;
        }
    );

    return {
        {"op", "register_param_callback"},
        {"mode", mode},
        {"reject", payload["reject"]},
        {"registered", true},
    };
}

json exec_register_pre_set_callback(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload) {
        throw BundleError("register_pre_set_callback: missing payload");
    }

    const json& payload = *op.payload;
    std::string mode = payload.value("mode", "");
    if (mode != "remove_names") {
        throw BundleError("register_pre_set_callback: unsupported mode '" + mode + "'");
    }
    if (!payload.contains("remove") || !payload["remove"].is_array()) {
        throw BundleError("register_pre_set_callback: payload.remove must be an array");
    }

    std::set<std::string> remove_names;
    for (const auto& item : payload["remove"]) {
        if (!item.is_string()) {
            throw BundleError("register_pre_set_callback: payload.remove[] must be strings");
        }
        remove_names.insert(item.get<std::string>());
    }

    if (st.pre_set_params_cb_handle) {
        st.node->remove_pre_set_parameters_callback(st.pre_set_params_cb_handle.get());
        st.pre_set_params_cb_handle.reset();
    }

    st.pre_set_params_cb_handle = st.node->add_pre_set_parameters_callback(
        [remove_names](std::vector<rclcpp::Parameter>& params) {
            params.erase(
                std::remove_if(
                    params.begin(),
                    params.end(),
                    [&remove_names](const rclcpp::Parameter& p) {
                        return remove_names.count(p.get_name()) > 0;
                    }
                ),
                params.end()
            );
        }
    );

    return {
        {"op", "register_pre_set_callback"},
        {"mode", mode},
        {"remove", payload["remove"]},
        {"registered", true},
    };
}

json exec_observe_param_events(
    TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op
) {
    uint64_t wait_ms = 100;
    if (op.ms) {
        wait_ms = *op.ms;
    } else if (op.payload && op.payload->contains("ms")) {
        wait_ms = (*op.payload)["ms"].get<uint64_t>();
    }

    if (!st.param_event_sub) {
        st.param_event_sub = st.node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events",
            rclcpp::ParameterEventsQoS(),
            [&st, scenario_id](const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
                json ev = {
                    {"type", "parameter_event"},
                    {"scenario_id", scenario_id},
                    {"node", msg->node},
                    {"stamp", {{"sec", msg->stamp.sec}, {"nanosec", msg->stamp.nanosec}}},
                    {"new_parameters", json::array()},
                    {"changed_parameters", json::array()},
                    {"deleted_parameters", json::array()},
                };

                for (const auto& p : msg->new_parameters) {
                    ev["new_parameters"].push_back(parameter_to_json(p));
                }
                for (const auto& p : msg->changed_parameters) {
                    ev["changed_parameters"].push_back(parameter_to_json(p));
                }
                for (const auto& p : msg->deleted_parameters) {
                    ev["deleted_parameters"].push_back(parameter_to_json(p));
                }

                std::lock_guard<std::mutex> lock(st.param_event_buffer_mutex);
                st.pending_param_events.push_back(std::move(ev));
            }
        );
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (st.executor) {
            st.executor->spin_some(std::chrono::milliseconds(5));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    size_t count = drain_parameter_event_buffer(w, st);

    return {
        {"op", "observe_param_events"},
        {"window_ms", wait_ms},
        {"count", count},
    };
}

json exec_list_params(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    std::vector<std::string> prefixes;
    uint64_t depth = 0;

    if (op.payload) {
        if (op.payload->contains("prefixes")) {
            if (!(*op.payload)["prefixes"].is_array()) {
                throw BundleError("list_params: payload.prefixes must be an array");
            }
            for (const auto& p : (*op.payload)["prefixes"]) {
                if (!p.is_string()) {
                    throw BundleError("list_params: payload.prefixes[] must be strings");
                }
                prefixes.push_back(p.get<std::string>());
            }
        }
        if (op.payload->contains("depth")) {
            if (!(*op.payload)["depth"].is_number_unsigned()) {
                throw BundleError("list_params: payload.depth must be an unsigned integer");
            }
            depth = (*op.payload)["depth"].get<uint64_t>();
        }
    }

    auto listed = st.node->list_parameters(prefixes, depth);
    return {
        {"op", "list_params"},
        {"prefixes_in", prefixes},
        {"depth", depth},
        {"names", listed.names},
        {"prefixes", listed.prefixes},
        {"count", listed.names.size()},
    };
}

json exec_get_parameter_types(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload || !op.payload->contains("names") || !(*op.payload)["names"].is_array()) {
        throw BundleError("get_parameter_types: payload.names must be an array");
    }

    std::vector<std::string> names;
    for (const auto& item : (*op.payload)["names"]) {
        if (!item.is_string()) {
            throw BundleError("get_parameter_types: payload.names[] must be strings");
        }
        names.push_back(item.get<std::string>());
    }

    json result = {
        {"op", "get_parameter_types"},
        {"successful", true},
        {"count", 0},
        {"types", json::array()},
    };
    try {
        auto types = st.node->get_parameter_types(names);
        result["count"] = types.size();
        for (const auto& t : types) {
            result["types"].push_back(static_cast<int>(t));
        }
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        result["successful"] = false;
        result["reason"] = "NotDeclared";
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = std::string("Exception: ") + e.what();
    }
    return result;
}

json exec_concurrent_read_during_batch(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload) {
        throw BundleError("concurrent_read_during_batch: missing payload");
    }
    if (!op.payload->contains("target_params") || !(*op.payload)["target_params"].is_array()) {
        throw BundleError("concurrent_read_during_batch: payload.target_params must be an array");
    }
    if (!op.payload->contains("batch") || !(*op.payload)["batch"].is_array()) {
        throw BundleError("concurrent_read_during_batch: payload.batch must be an array");
    }

    std::vector<std::string> targets;
    for (const auto& item : (*op.payload)["target_params"]) {
        if (!item.is_string()) {
            throw BundleError("concurrent_read_during_batch: payload.target_params[] must be strings");
        }
        targets.push_back(item.get<std::string>());
    }

    std::vector<rclcpp::Parameter> batch = build_parameter_list(
        (*op.payload)["batch"], "concurrent_read_during_batch"
    );

    std::map<std::string, std::string> old_values;
    for (const auto& name : targets) {
        old_values[name] = parameter_to_string(st.node->get_parameter(name));
    }

    std::map<std::string, std::string> new_values = old_values;
    for (const auto& p : batch) {
        new_values[p.get_name()] = parameter_to_string(p);
    }

    std::atomic<bool> done{false};
    std::atomic<bool> has_snapshot{false};
    std::vector<std::map<std::string, std::string>> snapshots;
    std::mutex snap_mutex;

    std::thread reader([&]() {
        while (!done.load(std::memory_order_relaxed)) {
            std::map<std::string, std::string> snap;
            auto params = st.node->get_parameters(targets);
            for (size_t idx = 0; idx < targets.size() && idx < params.size(); ++idx) {
                snap[targets[idx]] = parameter_to_string(params[idx]);
            }
            {
                std::lock_guard<std::mutex> lock(snap_mutex);
                snapshots.push_back(std::move(snap));
            }
            has_snapshot.store(true, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });

    // Ensure at least one pre-write snapshot to avoid nondeterministic empty samples.
    for (int i = 0; i < 200 && !has_snapshot.load(std::memory_order_relaxed); ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    auto results = st.node->set_parameters(batch);
    done.store(true, std::memory_order_relaxed);
    reader.join();

    bool partial_observed = false;
    for (const auto& snap : snapshots) {
        size_t old_count = 0;
        size_t new_count = 0;
        for (const auto& name : targets) {
            auto it = snap.find(name);
            if (it == snap.end()) {
                continue;
            }
            if (it->second == old_values[name]) {
                old_count++;
            }
            if (it->second == new_values[name]) {
                new_count++;
            }
        }
        if (old_count > 0 && new_count > 0) {
            partial_observed = true;
            break;
        }
    }

    bool all_successful = true;
    for (const auto& r : results) {
        if (!r.successful) {
            all_successful = false;
            break;
        }
    }

    return {
        {"op", "concurrent_read_during_batch"},
        {"batch_count", batch.size()},
        {"snapshot_count", snapshots.size()},
        {"partial_state_observed", partial_observed},
        {"successful", all_successful},
    };
}

json exec_describe_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("name"))
        throw BundleError("describe_param: payload missing 'name'");

    std::string name = (*op.payload)["name"];

    json result;
    result["op"] = "describe_param";
    result["name"] = name;

    rcl_interfaces::msg::ParameterDescriptor desc;

    try {
        auto descriptors = st.node->describe_parameters({name});
        if (descriptors.empty()) {
             throw BundleError("describe_parameters returned empty vector");
        }
        desc = descriptors[0];
        result["successful"] = true;
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = e.what();
        // Fallback to default constructed descriptor (type=0/NOT_SET)
    }

    result["descriptor"] = json::object();
    result["descriptor"]["type"] = desc.type;
    result["descriptor"]["read_only"] = desc.read_only;
    result["descriptor"]["dynamic_typing"] = desc.dynamic_typing;

    return result;
}

json exec_undeclare_param(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("name"))
        throw BundleError("undeclare_param: payload missing 'name'");

    std::string name = (*op.payload)["name"];
    json result;
    result["op"] = "undeclare_param";
    result["name"] = name;

    try {
        st.node->undeclare_parameter(name);
        result["successful"] = true;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        result["successful"] = false;
        result["reason"] = "NotDeclared";
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = std::string("Exception: ") + e.what();
    }

    return result;
}

} // namespace backend_prod::ops
