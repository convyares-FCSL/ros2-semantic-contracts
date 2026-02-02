#include "utils/trace.hpp"
#include "internal/errors.hpp"

namespace backend_prod {

TraceWriter::TraceWriter(const std::string& path, const std::string& run_id)
    : run_id_(run_id) {
    ofs_.open(path, std::ios::trunc);
    if (!ofs_.is_open()) {
        throw SystemError("failed to open trace file: " + path);
    }
    t0_ = std::chrono::steady_clock::now();
}

TraceWriter::~TraceWriter() {
    if (ofs_.is_open()) {
        ofs_.flush();
    }
}

void TraceWriter::emit(json event) {
    auto now = std::chrono::steady_clock::now();
    uint64_t t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - t0_).count();

    // Envelope fields first, then merge event body.
    event["version"] = version_;
    event["run_id"] = run_id_;
    event["t_ns"] = t_ns;
    event["sequence"] = seq_++;

    ofs_ << event.dump() << '\n';
    ofs_.flush();
}

} // namespace backend_prod
