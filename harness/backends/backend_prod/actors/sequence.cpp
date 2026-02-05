/// @file sequence.cpp
/// @brief H00 peer: action server with a fixed accept/reject sequence.
///
/// Goal 1 → ACCEPT_AND_EXECUTE.  Goal 2 → REJECT.
/// The counter is intrinsic to the process; it resets on restart.
/// No environment variables or scenario parameters affect the sequence.
///
/// Sentinel protocol is identical to reject_always (see that file for details).

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <test_msgs/action/fibonacci.hpp>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <unistd.h>

using FibAction = test_msgs::action::Fibonacci;

static std::string g_sentinel_path;
static std::atomic<bool> g_shutdown{false};
static std::atomic<int> g_goal_count{0};

static void cleanup_sentinel() {
    if (!g_sentinel_path.empty()) {
        (void)unlink(g_sentinel_path.c_str());
    }
}

static void signal_handler(int /*sig*/) {
    g_shutdown.store(true, std::memory_order_relaxed);
}

int main(int /*argc*/, char** /*argv*/) {
    // --- Sentinel path from RUN_ID ---
    const char* env_run_id = std::getenv("BACKEND_PROD_RUN_ID");
    std::string run_id = env_run_id ? env_run_id : "run_prod";
    g_sentinel_path = "/tmp/peer_ready_" + run_id;

    // --- Atomic-create sentinel (uniqueness check) ---
    int fd = open(g_sentinel_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        std::cerr << "peer_sequence: sentinel already exists at "
                  << g_sentinel_path << " — aborting\n";
        return 4;
    }
    close(fd);
    (void)unlink(g_sentinel_path.c_str());

    // --- Signal handling ---
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    // --- ROS 2 init ---
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("peer_sequence");

    // --- Action server (sequence-based) ---
    auto server = rclcpp_action::create_server<FibAction>(
        node,
        "test_action",
        // goal callback — counter-driven sequence
        [](const rclcpp_action::GoalUUID&,
           std::shared_ptr<const FibAction::Goal>) {
            int n = g_goal_count.fetch_add(1, std::memory_order_relaxed) + 1;
            if (n <= 1) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
            return rclcpp_action::GoalResponse::REJECT;
        },
        // cancel callback
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // execute callback — trivially complete accepted goals
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>> handle) {
            auto result = std::make_shared<FibAction::Result>();
            result->sequence = {};
            handle->succeed(result);
        });

    // --- Sentinel: visible create after server registration ---
    fd = open(g_sentinel_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        std::cerr << "peer_sequence: sentinel race at " << g_sentinel_path << "\n";
        rclcpp::shutdown();
        return 4;
    }
    close(fd);

    std::cerr << "peer_sequence: ready (sentinel " << g_sentinel_path << ")\n";

    // --- Spin until shutdown ---
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!g_shutdown.load(std::memory_order_relaxed) && rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(50));
    }

    cleanup_sentinel();
    rclcpp::shutdown();
    return 0;
}
