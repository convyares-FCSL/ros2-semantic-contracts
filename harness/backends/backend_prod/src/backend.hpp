/// @file backend.hpp
/// @brief Internal backend implementation header (not part of public API).
#ifndef BACKEND_PROD_SRC_BACKEND_HPP_
#define BACKEND_PROD_SRC_BACKEND_HPP_

#include "internal/types.hpp"
#include "internal/errors.hpp"
#include "utils/trace.hpp"

#include <memory>
namespace rclcpp { class Node; }
namespace rclcpp_lifecycle { class LifecycleNode; }

namespace backend_prod {

struct NodeContext {
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node;
};

// Op execution dispatch (called from backend.cpp)
void exec_op(TraceWriter& w, NodeContext& ctx, const std::string& scenario_id, const std::string& op_id, const Op& op);

} // namespace backend_prod

#endif // BACKEND_PROD_SRC_BACKEND_HPP_
