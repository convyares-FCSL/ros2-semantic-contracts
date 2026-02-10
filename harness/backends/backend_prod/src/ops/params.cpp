#include "../backend.hpp"
#include "internal/errors.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <set>
#include <thread>

namespace backend_prod::ops {

// payload contract: "name", "type", and "value" required (checked by caller).
static rclcpp::Parameter make_parameter(const std::string& name, const json& payload) {
    if (!payload.contains("type") || !payload["type"].is_string()) {
        throw BundleError("set_param: payload missing 'type'");
    }
    std::string type = payload["type"].get<std::string>();
    if (type == "integer")  return rclcpp::Parameter(name, payload["value"].get<int64_t>());
    if (type == "double")   return rclcpp::Parameter(name, payload["value"].get<double>());
    if (type == "string")   return rclcpp::Parameter(name, payload["value"].get<std::string>());
    if (type == "bool")     return rclcpp::Parameter(name, payload["value"].get<bool>());
    if (type == "byte_array") {
        return rclcpp::Parameter(name, payload["value"].get<std::vector<uint8_t>>());
    }
    if (type == "bool_array") {
        return rclcpp::Parameter(name, payload["value"].get<std::vector<bool>>());
    }
    if (type == "integer_array") {
        return rclcpp::Parameter(name, payload["value"].get<std::vector<int64_t>>());
    }
    if (type == "double_array") {
        return rclcpp::Parameter(name, payload["value"].get<std::vector<double>>());
    }
    if (type == "string_array") {
        return rclcpp::Parameter(name, payload["value"].get<std::vector<std::string>>());
    }
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
        if (!p.count("type") || !p["type"].is_string()) {
            throw BundleError(op_name + ": each param must have string 'type'");
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

static void apply_descriptor_json(
    const json& descriptor_json, rcl_interfaces::msg::ParameterDescriptor& desc, const std::string& op_name
) {
    if (!descriptor_json.is_object()) {
        throw BundleError(op_name + ": payload.descriptor must be an object");
    }

    if (descriptor_json.contains("read_only")) {
        if (!descriptor_json["read_only"].is_boolean()) {
            throw BundleError(op_name + ": payload.descriptor.read_only must be boolean");
        }
        desc.read_only = descriptor_json["read_only"].get<bool>();
    }
    if (descriptor_json.contains("dynamic_typing")) {
        if (!descriptor_json["dynamic_typing"].is_boolean()) {
            throw BundleError(op_name + ": payload.descriptor.dynamic_typing must be boolean");
        }
        desc.dynamic_typing = descriptor_json["dynamic_typing"].get<bool>();
    }
    if (descriptor_json.contains("type")) {
        if (!descriptor_json["type"].is_number_integer()) {
            throw BundleError(op_name + ": payload.descriptor.type must be integer");
        }
        desc.type = static_cast<uint8_t>(descriptor_json["type"].get<int>());
    }
    if (descriptor_json.contains("description")) {
        if (!descriptor_json["description"].is_string()) {
            throw BundleError(op_name + ": payload.descriptor.description must be string");
        }
        desc.description = descriptor_json["description"].get<std::string>();
    }
    if (descriptor_json.contains("additional_constraints")) {
        if (!descriptor_json["additional_constraints"].is_string()) {
            throw BundleError(op_name + ": payload.descriptor.additional_constraints must be string");
        }
        desc.additional_constraints = descriptor_json["additional_constraints"].get<std::string>();
    }
    if (descriptor_json.contains("integer_range")) {
        if (!descriptor_json["integer_range"].is_array()) {
            throw BundleError(op_name + ": payload.descriptor.integer_range must be array");
        }
        desc.integer_range.clear();
        for (const auto& range_spec : descriptor_json["integer_range"]) {
            if (!range_spec.is_object()) {
                throw BundleError(op_name + ": payload.descriptor.integer_range[] must be object");
            }
            if (
                !range_spec.contains("from_value") ||
                !range_spec.contains("to_value") ||
                !range_spec.contains("step")
            ) {
                throw BundleError(
                    op_name +
                    ": payload.descriptor.integer_range[] requires from_value,to_value,step"
                );
            }
            if (!range_spec["from_value"].is_number_integer() || !range_spec["to_value"].is_number_integer()) {
                throw BundleError(
                    op_name + ": payload.descriptor.integer_range[].from_value/to_value must be integers"
                );
            }
            if (!range_spec["step"].is_number_integer() && !range_spec["step"].is_number_unsigned()) {
                throw BundleError(op_name + ": payload.descriptor.integer_range[].step must be integer");
            }
            const int64_t step_signed = range_spec["step"].get<int64_t>();
            if (step_signed < 0) {
                throw BundleError(op_name + ": payload.descriptor.integer_range[].step must be non-negative");
            }
            rcl_interfaces::msg::IntegerRange range;
            range.from_value = range_spec["from_value"].get<int64_t>();
            range.to_value = range_spec["to_value"].get<int64_t>();
            range.step = static_cast<uint64_t>(step_signed);
            desc.integer_range.push_back(range);
        }
    }
    if (descriptor_json.contains("floating_point_range")) {
        if (!descriptor_json["floating_point_range"].is_array()) {
            throw BundleError(op_name + ": payload.descriptor.floating_point_range must be array");
        }
        desc.floating_point_range.clear();
        for (const auto& range_spec : descriptor_json["floating_point_range"]) {
            if (!range_spec.is_object()) {
                throw BundleError(op_name + ": payload.descriptor.floating_point_range[] must be object");
            }
            if (
                !range_spec.contains("from_value") ||
                !range_spec.contains("to_value") ||
                !range_spec.contains("step")
            ) {
                throw BundleError(
                    op_name +
                    ": payload.descriptor.floating_point_range[] requires from_value,to_value,step"
                );
            }
            if (
                !range_spec["from_value"].is_number() ||
                !range_spec["to_value"].is_number() ||
                !range_spec["step"].is_number()
            ) {
                throw BundleError(
                    op_name +
                    ": payload.descriptor.floating_point_range[].from_value/to_value/step must be numbers"
                );
            }
            rcl_interfaces::msg::FloatingPointRange range;
            range.from_value = range_spec["from_value"].get<double>();
            range.to_value = range_spec["to_value"].get<double>();
            range.step = range_spec["step"].get<double>();
            desc.floating_point_range.push_back(range);
        }
    }
}

static json descriptor_to_json(const rcl_interfaces::msg::ParameterDescriptor& desc) {
    json descriptor = {
        {"type", desc.type},
        {"read_only", desc.read_only},
        {"dynamic_typing", desc.dynamic_typing},
        {"description", desc.description},
        {"additional_constraints", desc.additional_constraints},
        {"integer_range", json::array()},
        {"floating_point_range", json::array()},
    };
    for (const auto& range : desc.integer_range) {
        descriptor["integer_range"].push_back(
            {
                {"from_value", range.from_value},
                {"to_value", range.to_value},
                {"step", range.step},
            }
        );
    }
    for (const auto& range : desc.floating_point_range) {
        descriptor["floating_point_range"].push_back(
            {
                {"from_value", range.from_value},
                {"to_value", range.to_value},
                {"step", range.step},
            }
        );
    }
    return descriptor;
}

static std::string cli_assignment_from_json_value(const std::string& name, const json& value) {
    if (value.is_boolean()) {
        return name + ":=" + std::string(value.get<bool>() ? "true" : "false");
    }
    if (value.is_number_integer()) {
        return name + ":=" + std::to_string(value.get<int64_t>());
    }
    if (value.is_number_float()) {
        return name + ":=" + value.dump();
    }
    if (value.is_string()) {
        return name + ":=" + value.get<std::string>();
    }
    throw BundleError("probe_cli_typed_no_default_declare: payload.cli_value must be bool/int/double/string");
}

static void parameter_to_observed_value(
    const rclcpp::Parameter& param, json& out_value, std::string& out_type, bool& out_set
) {
    out_set = true;
    switch (param.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            out_value = param.as_bool();
            out_type = "bool";
            return;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            out_value = param.as_int();
            out_type = "integer";
            return;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            out_value = param.as_double();
            out_type = "double";
            return;
        case rclcpp::ParameterType::PARAMETER_STRING:
            out_value = param.as_string();
            out_type = "string";
            return;
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
            out_set = false;
            out_type = "NOT_SET";
            return;
        default:
            out_type = "unsupported";
            out_value = parameter_to_string(param);
            return;
    }
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

json exec_set_param_atomic(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.payload || !op.payload->count("name")) {
        throw BundleError("set_param_atomic: payload missing 'name'");
    }
    if (!op.payload->count("value")) {
        throw BundleError("set_param_atomic: payload missing 'value'");
    }
    std::string name = (*op.payload)["name"];

    json result;
    result["op"] = "set_param_atomic";
    result["name"] = name;

    try {
        std::vector<rclcpp::Parameter> params;
        params.push_back(make_parameter(name, *op.payload));
        rcl_interfaces::msg::SetParametersResult res = st.node->set_parameters_atomically(params);
        result["successful"] = res.successful;
        if (!res.successful && !res.reason.empty()) {
            result["reason"] = res.reason;
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
        apply_descriptor_json((*op.payload)["descriptor"], desc, "declare_param");
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

        // Echo full descriptor for trace inspection.
        result["descriptor"] = descriptor_to_json(desc);
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
    if (mode != "reject_names" && mode != "assert_storage_old_value") {
        throw BundleError("register_param_callback: unsupported mode '" + mode + "'");
    }
    std::set<std::string> reject_names;
    std::string target_name;
    std::string expected_old_value;

    if (mode == "reject_names") {
        if (!payload.contains("reject") || !payload["reject"].is_array()) {
            throw BundleError("register_param_callback: payload.reject must be an array");
        }
        for (const auto& item : payload["reject"]) {
            if (!item.is_string()) {
                throw BundleError("register_param_callback: payload.reject[] must be strings");
            }
            reject_names.insert(item.get<std::string>());
        }
    } else if (mode == "assert_storage_old_value") {
        if (!payload.contains("target") || !payload["target"].is_string()) {
            throw BundleError("register_param_callback: payload.target must be a string");
        }
        if (!payload.contains("expected_old_value") || !payload["expected_old_value"].is_string()) {
            throw BundleError("register_param_callback: payload.expected_old_value must be a string");
        }
        target_name = payload["target"].get<std::string>();
        expected_old_value = payload["expected_old_value"].get<std::string>();
    }

    if (st.on_set_params_cb_handle) {
        st.node->remove_on_set_parameters_callback(st.on_set_params_cb_handle.get());
        st.on_set_params_cb_handle.reset();
    }

    if (mode == "reject_names") {
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

    auto node = st.node;
    st.on_set_params_cb_handle = st.node->add_on_set_parameters_callback(
        [node, target_name, expected_old_value](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;

            bool target_in_request = false;
            for (const auto& p : params) {
                if (p.get_name() == target_name) {
                    target_in_request = true;
                    break;
                }
            }

            if (!target_in_request) {
                return res;
            }

            std::string observed = "NOT_SET";
            try {
                auto current = node->get_parameter(target_name);
                observed = parameter_to_string(current);
            } catch (const rclcpp::exceptions::ParameterUninitializedException&) {
                observed = "NOT_SET";
            } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
                observed = "NOT_SET";
            }

            if (observed != expected_old_value) {
                res.successful = false;
                res.reason = "StorageNotOld:" + target_name;
            }
            return res;
        }
    );

    return {
        {"op", "register_param_callback"},
        {"mode", mode},
        {"target", target_name},
        {"expected_old_value", expected_old_value},
        {"registered", true},
    };
}

json exec_register_pre_set_callback(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
#if !BACKEND_PROD_HAS_PRE_SET_CALLBACK
    (void)st;
    (void)op;
    return {
        {"op", "register_pre_set_callback"},
        {"registered", false},
        {"reason", "UnsupportedOnThisDistro"},
    };
#else
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
#endif
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
        {"names", names},
        {"request_count", names.size()},
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
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        result["successful"] = false;
        result["reason"] = "NotDeclared";
        result["reason_raw"] = e.what();
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = std::string("Exception: ") + e.what();
        result["reason_raw"] = e.what();
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

    std::string api = "non_atomic";
    if (op.payload->contains("api")) {
        if (!(*op.payload)["api"].is_string()) {
            throw BundleError("concurrent_read_during_batch: payload.api must be a string");
        }
        api = (*op.payload)["api"].get<std::string>();
    }
    if (api != "non_atomic" && api != "atomic") {
        throw BundleError("concurrent_read_during_batch: payload.api must be 'non_atomic' or 'atomic'");
    }

    int trial_count = 1;
    if (op.payload->contains("trials")) {
        if (!(*op.payload)["trials"].is_number_integer()) {
            throw BundleError("concurrent_read_during_batch: payload.trials must be an integer");
        }
        trial_count = (*op.payload)["trials"].get<int>();
        if (trial_count < 1) {
            throw BundleError("concurrent_read_during_batch: payload.trials must be >= 1");
        }
    }

    int min_snapshots = 1;
    if (op.payload->contains("min_snapshots")) {
        if (!(*op.payload)["min_snapshots"].is_number_integer()) {
            throw BundleError("concurrent_read_during_batch: payload.min_snapshots must be an integer");
        }
        min_snapshots = (*op.payload)["min_snapshots"].get<int>();
        if (min_snapshots < 1) {
            throw BundleError("concurrent_read_during_batch: payload.min_snapshots must be >= 1");
        }
    }

    int reader_sleep_us = 50;
    if (op.payload->contains("reader_sleep_us")) {
        if (!(*op.payload)["reader_sleep_us"].is_number_integer()) {
            throw BundleError("concurrent_read_during_batch: payload.reader_sleep_us must be an integer");
        }
        reader_sleep_us = (*op.payload)["reader_sleep_us"].get<int>();
        if (reader_sleep_us < 0) {
            throw BundleError("concurrent_read_during_batch: payload.reader_sleep_us must be >= 0");
        }
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

    std::map<std::string, rclcpp::Parameter> old_params;
    std::map<std::string, std::string> old_values;
    for (const auto& name : targets) {
        auto current = st.node->get_parameter(name);
        old_params[name] = current;
        old_values[name] = parameter_to_string(current);
    }

    std::map<std::string, std::string> new_values = old_values;
    for (const auto& p : batch) {
        new_values[p.get_name()] = parameter_to_string(p);
    }

    std::vector<rclcpp::Parameter> revert_batch;
    revert_batch.reserve(targets.size());
    for (const auto& name : targets) {
        revert_batch.push_back(old_params[name]);
    }

    bool all_successful = true;
    bool partial_observed = false;
    int mixed_snapshot_count = 0;
    int mixed_trial_count = 0;
    size_t snapshot_count = 0;

    for (int trial = 0; trial < trial_count; ++trial) {
        auto reset_result = st.node->set_parameters_atomically(revert_batch);
        if (!reset_result.successful) {
            all_successful = false;
            break;
        }

        std::atomic<bool> done{false};
        std::atomic<bool> has_snapshot{false};
        std::vector<std::map<std::string, std::string>> trial_snapshots;
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
                    trial_snapshots.push_back(std::move(snap));
                }
                has_snapshot.store(true, std::memory_order_relaxed);
                if (reader_sleep_us > 0) {
                    std::this_thread::sleep_for(std::chrono::microseconds(reader_sleep_us));
                } else {
                    std::this_thread::yield();
                }
            }
        });

        // Ensure at least one pre-write snapshot to avoid nondeterministic empty samples.
        for (int i = 0; i < 200 && !has_snapshot.load(std::memory_order_relaxed); ++i) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }

        if (api == "atomic") {
            auto one = st.node->set_parameters_atomically(batch);
            if (!one.successful) {
                all_successful = false;
            }
        } else {
            auto responses = st.node->set_parameters(batch);
            for (const auto& r : responses) {
                if (!r.successful) {
                    all_successful = false;
                    break;
                }
            }
        }

        done.store(true, std::memory_order_relaxed);
        reader.join();

        bool mixed_in_trial = false;
        for (const auto& snap : trial_snapshots) {
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
                mixed_in_trial = true;
                mixed_snapshot_count++;
            }
        }

        if (mixed_in_trial) {
            mixed_trial_count++;
        }

        snapshot_count += trial_snapshots.size();
    }

    const bool sample_sufficient = static_cast<int>(snapshot_count) >= min_snapshots;

    return {
        {"op", "concurrent_read_during_batch"},
        {"batch_count", batch.size()},
        {"api", api},
        {"trial_count", trial_count},
        {"min_snapshots", min_snapshots},
        {"reader_sleep_us", reader_sleep_us},
        {"snapshot_count", snapshot_count},
        {"sample_sufficient", sample_sufficient},
        {"mixed_snapshot_count", mixed_snapshot_count},
        {"mixed_trial_count", mixed_trial_count},
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

    result["descriptor"] = descriptor_to_json(desc);

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

json exec_probe_startup_declare_events(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload) {
        throw BundleError("probe_startup_declare_events: missing payload");
    }
    if (!op.payload->contains("node_name") || !(*op.payload)["node_name"].is_string()) {
        throw BundleError("probe_startup_declare_events: payload.node_name must be string");
    }
    if (!op.payload->contains("parameter_overrides") || !(*op.payload)["parameter_overrides"].is_array()) {
        throw BundleError("probe_startup_declare_events: payload.parameter_overrides must be array");
    }

    const std::string node_name = (*op.payload)["node_name"].get<std::string>();
    bool auto_declare = true;
    bool allow_undeclared = false;
    if (op.payload->contains("automatically_declare_parameters_from_overrides")) {
        if (!(*op.payload)["automatically_declare_parameters_from_overrides"].is_boolean()) {
            throw BundleError(
                "probe_startup_declare_events: payload.automatically_declare_parameters_from_overrides must be boolean"
            );
        }
        auto_declare = (*op.payload)["automatically_declare_parameters_from_overrides"].get<bool>();
    }
    if (op.payload->contains("allow_undeclared_parameters")) {
        if (!(*op.payload)["allow_undeclared_parameters"].is_boolean()) {
            throw BundleError(
                "probe_startup_declare_events: payload.allow_undeclared_parameters must be boolean"
            );
        }
        allow_undeclared = (*op.payload)["allow_undeclared_parameters"].get<bool>();
    }

    uint64_t window_ms = 400;
    if (op.payload->contains("window_ms")) {
        if (!(*op.payload)["window_ms"].is_number_unsigned()) {
            throw BundleError("probe_startup_declare_events: payload.window_ms must be unsigned integer");
        }
        window_ms = (*op.payload)["window_ms"].get<uint64_t>();
    }

    std::vector<rclcpp::Parameter> overrides = build_parameter_list(
        (*op.payload)["parameter_overrides"], "probe_startup_declare_events"
    );
    std::set<std::string> override_names;
    for (const auto& p : overrides) {
        override_names.insert(p.get_name());
    }

    std::mutex event_mutex;
    std::vector<size_t> keys_per_event;
    size_t changed_total = 0;
    size_t new_total = 0;
    size_t deleted_total = 0;

    auto sub = st.node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        rclcpp::ParameterEventsQoS(),
        [&event_mutex, &keys_per_event, &changed_total, &new_total, &deleted_total, node_name, override_names](
            const rcl_interfaces::msg::ParameterEvent::SharedPtr msg
        ) {
            const std::string expected_1 = node_name;
            const std::string expected_2 = "/" + node_name;
            const bool name_match =
                msg->node == expected_1 ||
                msg->node == expected_2 ||
                msg->node.ends_with(expected_2);
            if (!name_match) {
                return;
            }

            size_t new_in_scope = 0;
            size_t changed_in_scope = 0;
            size_t deleted_in_scope = 0;
            for (const auto& p : msg->new_parameters) {
                if (override_names.count(p.name) > 0) {
                    new_in_scope++;
                }
            }
            for (const auto& p : msg->changed_parameters) {
                if (override_names.count(p.name) > 0) {
                    changed_in_scope++;
                }
            }
            for (const auto& p : msg->deleted_parameters) {
                if (override_names.count(p.name) > 0) {
                    deleted_in_scope++;
                }
            }
            const size_t keys = new_in_scope + changed_in_scope + deleted_in_scope;
            if (keys == 0) {
                return;
            }

            std::lock_guard<std::mutex> lock(event_mutex);
            keys_per_event.push_back(keys);
            changed_total += changed_in_scope;
            new_total += new_in_scope;
            deleted_total += deleted_in_scope;
        }
    );

    rclcpp::NodeOptions probe_opts;
    probe_opts.allow_undeclared_parameters(allow_undeclared);
    probe_opts.automatically_declare_parameters_from_overrides(auto_declare);
    probe_opts.parameter_overrides(overrides);
    auto probe_node = std::make_shared<rclcpp::Node>(node_name, probe_opts);
    st.executor->add_node(probe_node);

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(window_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        st.executor->spin_some(std::chrono::milliseconds(5));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    st.executor->remove_node(probe_node);
    probe_node.reset();
    sub.reset();

    size_t max_keys_in_single_event = 0;
    for (const auto& keys : keys_per_event) {
        if (keys > max_keys_in_single_event) {
            max_keys_in_single_event = keys;
        }
    }

    return {
        {"op", "probe_startup_declare_events"},
        {"node_name", node_name},
        {"override_count", overrides.size()},
        {"window_ms", window_ms},
        {"event_count", keys_per_event.size()},
        {"max_keys_in_single_event", max_keys_in_single_event},
        {"batched_single_event_observed", max_keys_in_single_event >= overrides.size()},
        {"new_total", new_total},
        {"changed_total", changed_total},
        {"deleted_total", deleted_total},
    };
}

json exec_probe_startup_manual_declare_events(
    TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload) {
        throw BundleError("probe_startup_manual_declare_events: missing payload");
    }
    if (!op.payload->contains("node_name") || !(*op.payload)["node_name"].is_string()) {
        throw BundleError("probe_startup_manual_declare_events: payload.node_name must be string");
    }
    if (!op.payload->contains("parameters") || !(*op.payload)["parameters"].is_array()) {
        throw BundleError("probe_startup_manual_declare_events: payload.parameters must be array");
    }

    const std::string node_name = (*op.payload)["node_name"].get<std::string>();
    uint64_t window_ms = 500;
    if (op.payload->contains("window_ms")) {
        if (!(*op.payload)["window_ms"].is_number_unsigned()) {
            throw BundleError("probe_startup_manual_declare_events: payload.window_ms must be unsigned integer");
        }
        window_ms = (*op.payload)["window_ms"].get<uint64_t>();
    }

    std::vector<rclcpp::Parameter> parameters = build_parameter_list(
        (*op.payload)["parameters"], "probe_startup_manual_declare_events"
    );
    std::set<std::string> parameter_names;
    for (const auto& p : parameters) {
        parameter_names.insert(p.get_name());
    }

    std::mutex event_mutex;
    std::vector<size_t> keys_per_event;
    size_t changed_total = 0;
    size_t new_total = 0;
    size_t deleted_total = 0;

    auto sub = st.node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        rclcpp::ParameterEventsQoS(),
        [&event_mutex, &keys_per_event, &changed_total, &new_total, &deleted_total, node_name, parameter_names](
            const rcl_interfaces::msg::ParameterEvent::SharedPtr msg
        ) {
            const std::string expected_1 = node_name;
            const std::string expected_2 = "/" + node_name;
            const bool name_match =
                msg->node == expected_1 ||
                msg->node == expected_2 ||
                msg->node.ends_with(expected_2);
            if (!name_match) {
                return;
            }

            size_t new_in_scope = 0;
            size_t changed_in_scope = 0;
            size_t deleted_in_scope = 0;
            for (const auto& p : msg->new_parameters) {
                if (parameter_names.count(p.name) > 0) {
                    new_in_scope++;
                }
            }
            for (const auto& p : msg->changed_parameters) {
                if (parameter_names.count(p.name) > 0) {
                    changed_in_scope++;
                }
            }
            for (const auto& p : msg->deleted_parameters) {
                if (parameter_names.count(p.name) > 0) {
                    deleted_in_scope++;
                }
            }
            const size_t keys = new_in_scope + changed_in_scope + deleted_in_scope;
            if (keys == 0) {
                return;
            }

            std::lock_guard<std::mutex> lock(event_mutex);
            keys_per_event.push_back(keys);
            changed_total += changed_in_scope;
            new_total += new_in_scope;
            deleted_total += deleted_in_scope;
        }
    );

    rclcpp::NodeOptions probe_opts;
    probe_opts.allow_undeclared_parameters(false);
    probe_opts.automatically_declare_parameters_from_overrides(false);
    auto probe_node = std::make_shared<rclcpp::Node>(node_name, probe_opts);
    st.executor->add_node(probe_node);

    bool successful = true;
    json declare_errors = json::array();
    for (const auto& p : parameters) {
        try {
            probe_node->declare_parameter(p.get_name(), p.get_parameter_value());
        } catch (const std::exception& e) {
            successful = false;
            declare_errors.push_back(
                {
                    {"name", p.get_name()},
                    {"reason", e.what()},
                }
            );
        }
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(window_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        st.executor->spin_some(std::chrono::milliseconds(5));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    st.executor->remove_node(probe_node);
    probe_node.reset();
    sub.reset();

    size_t max_keys_in_single_event = 0;
    for (const auto& keys : keys_per_event) {
        if (keys > max_keys_in_single_event) {
            max_keys_in_single_event = keys;
        }
    }

    return {
        {"op", "probe_startup_manual_declare_events"},
        {"node_name", node_name},
        {"declare_count", parameters.size()},
        {"window_ms", window_ms},
        {"event_count", keys_per_event.size()},
        {"max_keys_in_single_event", max_keys_in_single_event},
        {"batched_single_event_observed", max_keys_in_single_event >= parameters.size()},
        {"new_total", new_total},
        {"changed_total", changed_total},
        {"deleted_total", deleted_total},
        {"successful", successful},
        {"declare_errors", declare_errors},
    };
}

json exec_probe_cli_typed_no_default_declare(
    TraceWriter& /*w*/, BackendState& /*st*/, const std::string& /*scenario_id*/, const Op& op
) {
    if (!op.payload) {
        throw BundleError("probe_cli_typed_no_default_declare: missing payload");
    }
    if (!op.payload->contains("param_name") || !(*op.payload)["param_name"].is_string()) {
        throw BundleError("probe_cli_typed_no_default_declare: payload.param_name must be string");
    }
    if (!op.payload->contains("descriptor_type") || !(*op.payload)["descriptor_type"].is_number_integer()) {
        throw BundleError("probe_cli_typed_no_default_declare: payload.descriptor_type must be integer");
    }
    if (!op.payload->contains("cli_value")) {
        throw BundleError("probe_cli_typed_no_default_declare: payload.cli_value is required");
    }

    const std::string param_name = (*op.payload)["param_name"].get<std::string>();
    const int descriptor_type = (*op.payload)["descriptor_type"].get<int>();
    const std::string node_name = op.payload->contains("node_name") && (*op.payload)["node_name"].is_string()
        ? (*op.payload)["node_name"].get<std::string>()
        : "probe_cli_typed_no_default";
    const std::string assignment = cli_assignment_from_json_value(param_name, (*op.payload)["cli_value"]);

    json result = {
        {"op", "probe_cli_typed_no_default_declare"},
        {"node_name", node_name},
        {"param_name", param_name},
        {"descriptor_type", descriptor_type},
        {"cli_assignment", assignment},
        {"successful", false},
        {"declared", false},
        {"set", false},
        {"type", "NOT_SET"},
    };

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    opts.arguments({"--ros-args", "-p", assignment});
    opts.allow_undeclared_parameters(false);
    opts.automatically_declare_parameters_from_overrides(false);

    try {
        auto probe_node = std::make_shared<rclcpp::Node>(node_name, opts);
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.type = static_cast<uint8_t>(descriptor_type);
        const auto declared_type = static_cast<rclcpp::ParameterType>(desc.type);
        probe_node->declare_parameter(param_name, declared_type, desc);

        result["declared"] = probe_node->has_parameter(param_name);
        const rclcpp::Parameter param = probe_node->get_parameter(param_name);
        bool set = false;
        std::string observed_type = "NOT_SET";
        json observed_value;
        parameter_to_observed_value(param, observed_value, observed_type, set);
        result["set"] = set;
        result["type"] = observed_type;
        if (set) {
            result["value"] = observed_value;
        }
        result["successful"] = true;
    } catch (const std::exception& e) {
        result["successful"] = false;
        result["reason"] = e.what();
    }

    return result;
}

} // namespace backend_prod::ops
