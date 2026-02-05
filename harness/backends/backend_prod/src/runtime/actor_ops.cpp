/// @file actor_lifecycle.cpp
/// @brief Implementations for start_actor and stop_actor operations.
#include "backend.hpp"
#include "internal/errors.hpp"
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <filesystem>
#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>

namespace backend_prod {
namespace runtime {

using json = nlohmann::json;

// Helper to find actor binary in the same directory as the current executable.
static std::string find_actor_binary(const std::string& actor_name) {
    char self_path[4096];
    ssize_t len = readlink("/proc/self/exe", self_path, sizeof(self_path)-1);
    if (len == -1) {
        throw SystemError("failed to resolve /proc/self/exe");
    }
    self_path[len] = '\0';
    std::filesystem::path bin_path(self_path);
    std::filesystem::path dir = bin_path.parent_path();
    std::filesystem::path actor = dir / actor_name;
    
    if (!std::filesystem::exists(actor)) {
        throw BundleError("actor binary not found: " + actor.string());
    }
    return actor.string();
}

json exec_start_actor(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.actor) throw BundleError("start_actor missing 'actor' field");
    std::string actor = *op.actor;
    
    if (st.pid_map.count(actor)) {
        throw BundleError("actor already running: " + actor);
    }

    std::string binary_path = find_actor_binary(actor);
    
    std::vector<std::string> cmd_args;
    cmd_args.push_back(actor);
    cmd_args.push_back("--ros-args");
    
    // Parameters
    json params = op.params.value_or(json::object());
    for (auto& [key, val] : params.items()) {
        cmd_args.push_back("-p");
        std::string param_str = key + ":=";
        if (val.is_string()) {
            param_str += val.get<std::string>();
        } else {
             param_str += val.dump();
        }
        cmd_args.push_back(param_str);
    }
    
    // Generate unique sentinel path
    const char* env_run_id = std::getenv("BACKEND_PROD_RUN_ID");
    std::string run_id = env_run_id ? env_run_id : "unknown";
    // Use unique filename to allow parallel/multiple actors
    std::string sentinel_path = "/tmp/actor_ready_" + run_id + "_" + actor + "_" + std::to_string(getpid());

    // Convert to char* array for execv
    std::vector<char*> argv;
    for (const auto& arg : cmd_args) {
        argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);
    
    pid_t pid = fork();
    if (pid < 0) {
        throw SystemError("fork failed");
    }
    
    if (pid == 0) {
        // Child process
        // Pass sentinel path via environment
        setenv("ACTOR_READY_PATH", sentinel_path.c_str(), 1);
        
        execv(binary_path.c_str(), argv.data());
        perror("execv failed");
        exit(1);
    }
    
    // Parent process
    st.pid_map[actor] = pid;
    
    // Wait for sentinel (timeout 5s)
    bool ready = false;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        if (std::filesystem::exists(sentinel_path)) {
            ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    if (!ready) {
        // Timeout: cleanup and fail
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        st.pid_map.erase(actor);
        throw SystemError("timed out waiting for actor sentinel: " + sentinel_path);
    }

    // Return details for op_end
    return {
        {"actor", actor},
        {"params", params},
        {"pid", pid},
        {"sentinel", sentinel_path}
    };
}

json exec_stop_actor(TraceWriter& /*w*/, BackendState& st, const std::string& /*scenario_id*/, const Op& op) {
    if (!op.actor) throw BundleError("stop_actor missing 'actor' field");
    std::string actor = *op.actor;
    
    if (!st.pid_map.count(actor)) {
        throw BundleError("actor not running: " + actor);
    }
    
    pid_t pid = st.pid_map[actor];
    
    kill(pid, SIGTERM);
    
    int status;
    waitpid(pid, &status, 0);
    
    st.pid_map.erase(actor);
    
    // Return details for op_end
    return {
        {"actor", actor},
        {"exit_code", WEXITSTATUS(status)}
    };
}

} // namespace runtime
} // namespace backend_prod
