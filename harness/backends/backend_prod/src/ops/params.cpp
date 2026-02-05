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

    try {
        st.node->declare_parameter<std::string>(name, "");
        result["declared"] = true;
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        result["declared"] = false;
        result["reason"] = "AlreadyDeclared";
    }

    return result;
}

} // namespace backend_prod::ops
