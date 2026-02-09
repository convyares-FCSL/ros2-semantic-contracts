/// @file generic.cpp
/// @brief Generic operations usable across all scenario types (wait, etc.).
#include "internal/types.hpp"
#include "internal/errors.hpp"
#include "status_drain.hpp"
#include <thread>
#include <chrono>

namespace backend_prod::ops {

/// Executes wait: sleep for op.ms milliseconds.
void exec_wait(TraceWriter& w, BackendState& st, const std::string& scenario_id, const Op& op) {
    if (!op.ms) {
        throw BundleError("wait: missing ms");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(*op.ms));

    // NEW: Drain status buffer after wait (status messages arrive during sleep)
    drain_status_buffer(w, st);
}

} // namespace backend_prod::ops
