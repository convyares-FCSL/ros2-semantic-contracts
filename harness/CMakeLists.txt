cmake_minimum_required(VERSION 3.16)
project(ros2_semantic_oracle_harness LANGUAGES CXX)

# ------------------------------------------------------------
# Toolchain / language
# ------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# Paths
# ------------------------------------------------------------
set(HARNESS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(INCLUDE_DIR   ${HARNESS_ROOT}/include)

# ------------------------------------------------------------
# Core oracle library (transport-agnostic)
# ------------------------------------------------------------
add_library(oracle_core STATIC
  src/oracle_core.cpp
)

target_include_directories(oracle_core
  PUBLIC
    ${INCLUDE_DIR}
)

# ------------------------------------------------------------
# Mechanistic oracle runner (A: stub / pre-ROS)
# ------------------------------------------------------------
add_executable(oracle_a
  src/oracle_a.cpp
)

target_link_libraries(oracle_a
  PRIVATE oracle_core
)

# ------------------------------------------------------------
# Trace auditor (prints PASS/FAIL summary)
# ------------------------------------------------------------
add_executable(audit_trace
  src/audit_trace.cpp
)

target_include_directories(audit_trace
  PRIVATE ${INCLUDE_DIR}
)

# ------------------------------------------------------------
# ROS adapter (optional, A-with-ROS)
# ------------------------------------------------------------
find_package(rclcpp QUIET)

if (rclcpp_FOUND)
  message(STATUS "rclcpp found: building oracle_a_ros")

  add_executable(oracle_a_ros
    src/oracle_a_ros.cpp
  )

  target_link_libraries(oracle_a_ros
    PRIVATE oracle_core rclcpp::rclcpp
  )

  target_compile_definitions(oracle_a_ros
    PRIVATE ORACLE_HAS_ROS=1
  )
else()
  message(STATUS "rclcpp not found: skipping oracle_a_ros target")
endif()
