/// @file status_drain.hpp
/// @brief Shared helper for draining pending status events
#ifndef BACKEND_PROD_OPS_STATUS_DRAIN_HPP_
#define BACKEND_PROD_OPS_STATUS_DRAIN_HPP_

#include "../backend.hpp"

namespace backend_prod::ops {

/// Drain pending status events and emit via TraceWriter
/// SAFETY: Only call when TraceWriter& w is valid (during op execution)
inline void drain_status_buffer(TraceWriter& w, BackendState& st) {
    std::lock_guard<std::mutex> lock(st.status_buffer_mutex);
    for (const auto& ev : st.pending_status_events) {
        w.emit(ev);  // TraceWriter adds sequence, t_ns, version, run_id
    }
    st.pending_status_events.clear();
}

} // namespace backend_prod::ops

#endif // BACKEND_PROD_OPS_STATUS_DRAIN_HPP_
