/// @file params.cpp
/// @brief Parameter operations.
#include "../backend.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace backend_prod::ops {

using json = nlohmann::json;

json exec_declare_param(NodeContext& ctx, const Op& op) {
    if (!op.payload || !op.payload->contains("name")) {
        return {{"successful", false}, {"error", "missing name"}};
    }
    std::string name = (*op.payload)["name"];

    try {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        
        if (ctx.lifecycle_node) {
            ctx.lifecycle_node->declare_parameter(name, rclcpp::ParameterValue(), descriptor);
        } else {
            ctx.node->declare_parameter(name, rclcpp::ParameterValue(), descriptor);
        }
        
        return {{"name", name}, {"successful", true}};
    } catch (const std::exception& e) {
        return {{"name", name}, {"successful", false}, {"error", e.what()}};
    }
}

json exec_set_params_batch(NodeContext& ctx, const Op& op) {
    if (!op.payload || !op.payload->contains("params")) {
        return {{"all_successful", false}, {"error", "missing params"}};
    }

    std::vector<rclcpp::Parameter> params;
    for (const auto& p : (*op.payload)["params"]) {
        std::string name = p["name"];
        std::string type = p.value("type", "string");
        auto val_json = p["value"];
        
        if (type == "string") {
            params.emplace_back(name, val_json.get<std::string>());
        } else if (type == "integer") {
            if (val_json.is_number()) params.emplace_back(name, val_json.get<int64_t>());
            else params.emplace_back(name, std::stol(val_json.get<std::string>()));
        } else if (type == "double") {
            if (val_json.is_number()) params.emplace_back(name, val_json.get<double>());
            else params.emplace_back(name, std::stod(val_json.get<std::string>()));
        } else if (type == "bool") {
             if (val_json.is_boolean()) params.emplace_back(name, val_json.get<bool>());
             else params.emplace_back(name, val_json.get<std::string>() == "true");
        } else {
             params.emplace_back(name, val_json.dump());
        }
    }

    try {
        rcl_interfaces::msg::SetParametersResult res;
        if (ctx.lifecycle_node) {
             res = ctx.lifecycle_node->set_parameters_atomically(params);
        } else {
             res = ctx.node->set_parameters_atomically(params);
        }
        
        bool all_successful = res.successful;
        json res_array = json::array();
        
        for (size_t i = 0; i < params.size(); ++i) {
            res_array.push_back(json{{"successful", res.successful}, {"reason", res.reason}});
        }

        return {
            {"all_successful", all_successful},
            {"results", res_array}
        };

    } catch (const std::exception& e) {
        return {{"all_successful", false}, {"error", e.what()}};
    }
}

} // namespace backend_prod::ops