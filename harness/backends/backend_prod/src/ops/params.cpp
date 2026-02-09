#include "../backend.hpp"
#include "internal/errors.hpp"
#include <rclcpp/rclcpp.hpp>

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

    const json& params_array = (*op.payload)["params"];
    if (!params_array.is_array())
        throw BundleError("set_params_batch: 'params' must be an array");

    std::vector<rclcpp::Parameter> params;
    for (const auto& p : params_array) {
        if (!p.count("name") || !p.count("value"))
            throw BundleError("set_params_batch: each param must have 'name' and 'value'");
        params.push_back(make_parameter(p["name"], p));
    }

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
