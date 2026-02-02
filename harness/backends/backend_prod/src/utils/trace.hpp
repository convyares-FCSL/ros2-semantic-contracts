/// @file trace.hpp
/// @brief TraceWriter for JSONL event emission.
#ifndef BACKEND_PROD_TRACE_HPP_
#define BACKEND_PROD_TRACE_HPP_

#include <string>
#include <fstream>
#include <chrono>
#include <nlohmann/json.hpp>

namespace backend_prod {

using json = nlohmann::json;

/// Writes JSONL trace with envelope fields (version, run_id, t_ns, sequence).
/// Thread-unsafe; single-writer model only.
class TraceWriter {
public:
    explicit TraceWriter(const std::string& path, const std::string& run_id = "run_prod");
    ~TraceWriter();

    /// Emit event with auto-generated envelope fields.
    void emit(json event);

    TraceWriter(const TraceWriter&) = delete;
    TraceWriter& operator=(const TraceWriter&) = delete;

private:
    std::ofstream ofs_;
    uint64_t seq_{0};
    std::chrono::steady_clock::time_point t0_;
    std::string version_{"0.1"};
    std::string run_id_{"run_prod"};
};

} // namespace backend_prod

#endif // BACKEND_PROD_TRACE_HPP_
