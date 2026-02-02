/// @file backend.hpp
/// @brief Public API for backend_prod.
#ifndef BACKEND_PROD_BACKEND_HPP_
#define BACKEND_PROD_BACKEND_HPP_

#include <string>

namespace backend_prod {

/// Execute scenario bundle and write trace.
/// @param bundle_path Path to scenarios JSON bundle.
/// @param trace_path Path to output JSONL trace.
/// @throws std::runtime_error on bundle/trace errors.
void run_backend(const std::string& bundle_path, const std::string& trace_path);

} // namespace backend_prod

#endif // BACKEND_PROD_BACKEND_HPP_
