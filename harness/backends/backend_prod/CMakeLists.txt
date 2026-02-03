cmake_minimum_required(VERSION 3.8)
project(backend_prod)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(backend_prod
  src/main.cpp
  src/backend.cpp
  src/utils/trace.cpp
  src/internal/bundle_parser.cpp
  src/internal/fs_utils.cpp
  src/ops/actions.cpp
  src/ops/params.cpp
  src/ops/lifecycle.cpp
  src/ops/generic.cpp
)

# Include paths: public headers, internal sources, and vendored 3rdparty.
target_include_directories(backend_prod
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include/3rdparty
)

target_compile_features(backend_prod PUBLIC cxx_std_20)
ament_target_dependencies(backend_prod rclcpp rclcpp_lifecycle)

# Public header purity check: ensures backend_prod public headers do not leak
# internal or 3rdparty deps such as nlohmann/json. This target compiles with
# ONLY public include paths - if it fails, a public header is leaking internals.
add_library(header_sanity OBJECT src/header_sanity.cpp)
target_include_directories(header_sanity
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(header_sanity PUBLIC cxx_std_20)

install(TARGETS backend_prod DESTINATION lib/${PROJECT_NAME})

# Install public headers
install(DIRECTORY include/ DESTINATION include)

ament_package()
