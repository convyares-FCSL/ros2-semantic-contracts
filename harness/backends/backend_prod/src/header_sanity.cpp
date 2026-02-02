/// @file header_sanity.cpp
/// @brief Public header purity check: ensures backend_prod public headers
///        compile standalone without internal or 3rdparty dependencies.
///
/// This compilation unit includes ONLY public headers from include/backend_prod/
/// and must compile without access to src/ or include/3rdparty/ paths.
/// If this fails to compile, a public header is leaking internal dependencies.

#include "backend_prod/backend.hpp"

// Minimal compilation check - no runtime code needed.
// The act of compiling this file proves public headers are self-contained.
