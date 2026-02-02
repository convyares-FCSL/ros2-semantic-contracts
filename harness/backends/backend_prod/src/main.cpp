/// @file main.cpp
/// @brief Entry point for backend_prod.
/// Contract: backend_prod <bundle_path> <trace_path>
/// Exit codes: 0 = success, 2 = usage/bundle error, 4 = system error.
#include <rclcpp/rclcpp.hpp>
#include "backend_prod/backend.hpp"
#include "internal/errors.hpp"
#include <iostream>
#include <exception>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: backend_prod <bundle_path> <trace_path>\n";
        return 2;
    }

    const std::string bundle_path = argv[1];
    const std::string trace_path = argv[2];

    rclcpp::init(argc, argv);

    try {
        backend_prod::run_backend(bundle_path, trace_path);
    } catch (const backend_prod::BundleError& e) {
        std::cerr << "bundle error: " << e.what() << '\n';
        rclcpp::shutdown();
        return 2;  // usage/config error
    } catch (const backend_prod::SystemError& e) {
        std::cerr << "system error: " << e.what() << '\n';
        rclcpp::shutdown();
        return 4;  // runtime/infra error
    } catch (const std::exception& e) {
        std::cerr << "unexpected error: " << e.what() << '\n';
        rclcpp::shutdown();
        return 4;  // treat unknown as system error
    }

    rclcpp::shutdown();
    return 0;
}
