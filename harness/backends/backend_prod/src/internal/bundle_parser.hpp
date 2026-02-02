/// @file bundle_parser.hpp
/// @brief Centralized bundle parsing and validation.
#ifndef BACKEND_PROD_BUNDLE_PARSER_HPP_
#define BACKEND_PROD_BUNDLE_PARSER_HPP_

#include "internal/types.hpp"
#include <string>

namespace backend_prod {

/// Parse and validate scenario bundle from file.
/// @param path Path to bundle JSON file.
/// @return Validated Bundle structure.
/// @throws BundleError for validation failures (missing keys, wrong types, malformed JSON).
/// @throws SystemError for file I/O failures.
Bundle parse_bundle(const std::string& path);

} // namespace backend_prod

#endif // BACKEND_PROD_BUNDLE_PARSER_HPP_
