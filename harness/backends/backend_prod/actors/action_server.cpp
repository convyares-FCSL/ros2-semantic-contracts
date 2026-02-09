/// @file action_server.cpp
/// @brief Generic, parameterised action server for A01/A02 and beyond.
///
/// Parameters:
///   - goal_response (string, default "accept"): "accept" or "reject".
///   - outcome_sequence (string, default "succeed"): Comma-separated list of
///     outcomes to attempt: "succeed", "abort", "cancel".
///   - execute_delay_ms (int, default 0): Milliseconds to "work" before processing
///     outcome_sequence. During this delay, goal stays EXECUTING and cancel requests
///     can arrive. The execute loop checks is_canceling() every 10ms during delay.
///
/// Behavior:
///   - Configured via ROS parameters at startup.
///   - Writes sentinel file to path defined by ACTOR_READY_PATH environment variable.
///   - Clean shutdown on SIGTERM.
///   - Cancel-aware: if cancel arrives during execute_delay, transitions to CANCELED.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <test_msgs/action/fibonacci.hpp>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using FibAction = test_msgs::action::Fibonacci;

static std::string g_sentinel_path;
static std::atomic<bool> g_shutdown{false};

static void cleanup_sentinel() {
    if (!g_sentinel_path.empty()) {
        (void)unlink(g_sentinel_path.c_str());
    }
}

static void signal_handler(int /*sig*/) {
    g_shutdown.store(true, std::memory_order_relaxed);
}

// Helper to split outcome_sequence string
static std::vector<std::string> split_outcome_sequence(const std::string& seq) {
    std::vector<std::string> tokens;
    std::stringstream ss(seq);
    std::string token;
    while (std::getline(ss, token, ',')) {
        // Trim whitespace if needed, though usually params are clean
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char** argv) {
    // --- Sentinel path ---
    const char* env_ready_path = std::getenv("ACTOR_READY_PATH");
    if (env_ready_path) {
        g_sentinel_path = env_ready_path;
    } else {
        const char* env_run_id = std::getenv("BACKEND_PROD_RUN_ID");
        std::string run_id = env_run_id ? env_run_id : "run_prod";
        g_sentinel_path = "/tmp/peer_ready_" + run_id;
    }

    // --- Atomic-create sentinel check ---
    int fd = open(g_sentinel_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        std::cerr << "action_server: sentinel collision " << g_sentinel_path << "\n";
        return 4;
    }
    close(fd);
    (void)unlink(g_sentinel_path.c_str());

    // --- Signal handling ---
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    // --- ROS 2 init ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("action_server");

    // --- Parameters ---
    node->declare_parameter("goal_response", "accept");
    node->declare_parameter("outcome_sequence", "succeed");
    node->declare_parameter("execute_delay_ms", 0);

    // --- Action server ---
    auto server = rclcpp_action::create_server<FibAction>(
        node,
        "test_action",
        // Goal Callback
        [node](const rclcpp_action::GoalUUID&, std::shared_ptr<const FibAction::Goal>) {
            std::string resp = node->get_parameter("goal_response").as_string();
            if (resp == "reject") {
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // Cancel Callback
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // Accepted Callback
        [node](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>& goal_handle) {
            std::string seq_str = node->get_parameter("outcome_sequence").as_string();
            std::vector<std::string> sequence = split_outcome_sequence(seq_str);
            int64_t delay_ms = node->get_parameter("execute_delay_ms").as_int();

            std::thread([goal_handle, sequence, delay_ms]() {
                auto result = std::make_shared<FibAction::Result>();
                result->sequence = {1, 1, 2, 3, 5};

                // Execute delay: sleep in small increments, checking for cancel
                const int64_t check_interval_ms = 10;
                int64_t elapsed_ms = 0;
                while (elapsed_ms < delay_ms) {
                    // Check if goal is being canceled
                    if (goal_handle->is_canceling()) {
                        goal_handle->canceled(result);
                        return;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
                    elapsed_ms += check_interval_ms;
                }

                // Final check after delay completes
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                    return;
                }

                // Process outcome sequence
                for (const auto& outcome : sequence) {
                    try {
                        if (outcome == "succeed") {
                            goal_handle->succeed(result);
                        } else if (outcome == "abort") {
                            goal_handle->abort(result);
                        } else if (outcome == "cancel") {
                            goal_handle->canceled(result);
                        }
                    } catch (...) {
                        // Ignore invalid transitions or errors from rclcpp
                    }
                    // Small yield to ensure state changes propagate if multiple attempts
                     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }).detach();
        }
    );

    // --- Create Sentinel ---
    fd = open(g_sentinel_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        rclcpp::shutdown();
        return 4;
    }
    close(fd);

    std::cerr << "action_server: ready\n";

    // --- Spin ---
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!g_shutdown.load(std::memory_order_relaxed) && rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(50));
    }

    cleanup_sentinel();
    rclcpp::shutdown();
    return 0;
}
