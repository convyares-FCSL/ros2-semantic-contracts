/// @file reject_always.cpp
/// @brief A01 peer: action server that unconditionally rejects every goal.
///
/// Behaviour is structural — no environment variables or scenario parameters
/// alter the decision.  The binary writes a sentinel file after the action
/// server is registered so the environment can enforce strict startup ordering.
///
/// Sentinel path:  /tmp/peer_ready_<RUN_ID>
/// Atomic create:  O_CREAT | O_EXCL — exits with error if the file already
///                 exists (catches same-RUN_ID races).
/// Cleanup:        sentinel is unlinked on SIGTERM / normal exit.

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

    // --- Atomic-create sentinel (fail if stale copy exists for same RUN_ID) ---
    int fd = open(g_sentinel_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        std::cerr << "peer_reject_always: sentinel already exists at "
                  << g_sentinel_path << " — aborting (possible RUN_ID collision)\n";
        return 4;
    }
    // Sentinel will be written (touched) after server registration; close for now.
    close(fd);
    // Remove it immediately — we will re-create atomically after server is up.
    // This two-phase approach: first check uniqueness, then defer the visible
    // creation to after registration.
    (void)unlink(g_sentinel_path.c_str());

    // --- Signal handling for clean shutdown ---
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    // --- ROS 2 init ---
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("peer_reject_always");

    // --- Action server (reject-always) ---
    auto server = rclcpp_action::create_server<FibAction>(
        node,
        "test_action",
        // goal callback — unconditional REJECT
        [](const rclcpp_action::GoalUUID&,
           std::shared_ptr<const FibAction::Goal>) {
            return rclcpp_action::GoalResponse::REJECT;
        },
        // cancel callback — not reachable for rejected goals, but required by API
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // execute callback — not reachable for rejected goals
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FibAction>>&) {
        });

    // --- Sentinel: atomic create now that server is registered ---
    // Re-use O_CREAT | O_EXCL; a race with another peer for the same RUN_ID
    // will be caught here.
    fd = open(g_sentinel_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0) {
        std::cerr << "peer_reject_always: sentinel race detected at "
                  << g_sentinel_path << "\n";
        rclcpp::shutdown();
        return 4;
    }
    close(fd);

    std::cerr << "peer_reject_always: ready (sentinel " << g_sentinel_path << ")\n";

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
