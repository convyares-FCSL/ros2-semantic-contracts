#include "../backend.hpp"
#include "internal/errors.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
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
    }

    // Parse default value (string only)
    std::string default_value = op.payload->contains("value") ?
        (*op.payload)["value"].get<std::string>() : "";

    try {
        st.node->declare_parameter<std::string>(name, default_value, desc);
        result["declared"] = true;

        // Echo descriptor for trace inspection
        if (desc.read_only) {
            result["descriptor"] = json::object();
            result["descriptor"]["read_only"] = true;
        }
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        result["declared"] = false;
        result["reason"] = "AlreadyDeclared";
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
    rclcpp::Parameter param = st.node->get_parameter(name);

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

    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
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
