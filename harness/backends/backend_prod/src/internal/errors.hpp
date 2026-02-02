/// @file errors.hpp
/// @brief Custom exception types for proper exit code semantics.
#ifndef BACKEND_PROD_ERRORS_HPP_
#define BACKEND_PROD_ERRORS_HPP_

#include <stdexcept>
#include <string>

namespace backend_prod {

/// Exception for bundle/configuration errors (exit code 2).
/// Use for: invalid bundle format, missing keys, bad ops, malformed JSON.
class BundleError : public std::runtime_error {
public:
    explicit BundleError(const std::string& msg) : std::runtime_error(msg) {}
};

/// Exception for system/runtime errors (exit code 4).
/// Use for: filesystem failures, ROS init failures, unexpected runtime errors.
class SystemError : public std::runtime_error {
public:
    explicit SystemError(const std::string& msg) : std::runtime_error(msg) {}
};

} // namespace backend_prod

#endif // BACKEND_PROD_ERRORS_HPP_
