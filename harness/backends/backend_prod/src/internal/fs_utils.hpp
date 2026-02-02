/// @file fs_utils.hpp
/// @brief Filesystem utilities for backend_prod.
#ifndef BACKEND_PROD_FS_UTILS_HPP_
#define BACKEND_PROD_FS_UTILS_HPP_

#include <string>

namespace backend_prod {

/// Ensure parent directory exists for given file path.
/// @param file_path Path to file whose parent directory should exist.
/// @throws SystemError if directory creation fails.
void ensure_parent_dir(const std::string& file_path);

} // namespace backend_prod

#endif // BACKEND_PROD_FS_UTILS_HPP_
