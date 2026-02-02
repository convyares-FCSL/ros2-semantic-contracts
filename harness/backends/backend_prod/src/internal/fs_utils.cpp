#include "internal/fs_utils.hpp"
#include "internal/errors.hpp"
#include <filesystem>

namespace backend_prod {

void ensure_parent_dir(const std::string& file_path) {
    std::filesystem::path p(file_path);
    if (!p.has_parent_path()) {
        return;  // No parent directory needed
    }
    
    try {
        std::filesystem::create_directories(p.parent_path());
    } catch (const std::filesystem::filesystem_error& e) {
        throw SystemError("failed to create directory for trace: " + std::string(e.what()));
    }
}

} // namespace backend_prod
