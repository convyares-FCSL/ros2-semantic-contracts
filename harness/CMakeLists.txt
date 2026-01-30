cmake_minimum_required(VERSION 3.16)
project(ros2_semantic_oracle_harness LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Toolchain / language settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Repository paths
# ---------------------------------------------------------------------------
set(HARNESS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(INCLUDE_DIR  ${HARNESS_ROOT}/include)

# ---------------------------------------------------------------------------
# Core oracle library (transport-agnostic)
# ---------------------------------------------------------------------------
add_library(oracle_core STATIC
  src/oracle_core.cpp
)

target_include_directories(oracle_core
  PUBLIC
    ${INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# Mechanistic oracle runner (A: stub backend)
# ---------------------------------------------------------------------------
add_executable(oracle_a
  src/oracle_a.cpp
)

target_link_libraries(oracle_a
  PRIVATE oracle_core
)

# ---------------------------------------------------------------------------
# Trace auditor (consumes JSONL trace and prints PASS/FAIL summary)
# ---------------------------------------------------------------------------
add_executable(audit_trace
  src/audit_trace.cpp
)

target_include_directories(audit_trace
  PRIVATE ${INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# ROS adapter runner (A: ROS backend) - optional
#
# This project is built with plain CMake (not ament). Some ROS packages do not
# export imported CMake targets consistently in this mode (e.g. interface-only
# packages such as example_interfaces). Therefore:
# - link rclcpp and rclcpp_action via exported targets
# - include example_interfaces via exported include-dir variables when present
# ---------------------------------------------------------------------------
find_package(rclcpp QUIET)
find_package(rclcpp_action QUIET)
find_package(example_interfaces QUIET)

if (rclcpp_FOUND AND rclcpp_action_FOUND AND example_interfaces_FOUND)
  message(STATUS "ROS found: building oracle_a_ros (rclcpp + rclcpp_action + example_interfaces)")

  add_executable(oracle_a_ros
    src/oracle_a_ros.cpp
    src/ros_backend.cpp
  )

  target_link_libraries(oracle_a_ros
    PRIVATE oracle_core rclcpp::rclcpp rclcpp_action::rclcpp_action
  )

  target_include_directories(oracle_a_ros
    PRIVATE
      ${INCLUDE_DIR}
  )

  # example_interfaces is typically interface-only; in plain CMake it may not
  # export an imported target. Prefer exported include variables if available.
  if (DEFINED example_interfaces_INCLUDE_DIRS)
    target_include_directories(oracle_a_ros PRIVATE ${example_interfaces_INCLUDE_DIRS})
  elseif (DEFINED example_interfaces_INCLUDE_DIR)
    target_include_directories(oracle_a_ros PRIVATE ${example_interfaces_INCLUDE_DIR})
  else()
    message(STATUS "example_interfaces include dirs not exported; relying on environment include paths")
  endif()

  target_compile_definitions(oracle_a_ros
    PRIVATE ORACLE_HAS_ROS=1
  )

  # --- Fix for link errors in plain CMake (non-ament) ---
  # The example_interfaces package typically does not export an imported target for its
  # shared type-support libraries when used via find_package() in plain CMake.
  # We must manually find and link the type-support library to resolve symbols like:
  # rosidl_typesupport_cpp::get_action_type_support_handle<...>()

  find_package(rosidl_typesupport_cpp QUIET)
  
  # 1. Core typesupport dispatch lib
  find_library(LIB_ROSIDL_TYPESUPPORT_CPP 
    NAMES rosidl_typesupport_cpp
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH
  )

  # 2. Generated typesupport for example_interfaces
  find_library(LIB_EXAMPLE_INTERFACES_TYPESUPPORT_CPP
    NAMES example_interfaces__rosidl_typesupport_cpp
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH
  )

  if (LIB_ROSIDL_TYPESUPPORT_CPP AND LIB_EXAMPLE_INTERFACES_TYPESUPPORT_CPP)
    message(STATUS "  Found typesupport libs for plain CMake linking:")
    message(STATUS "    ${LIB_ROSIDL_TYPESUPPORT_CPP}")
    message(STATUS "    ${LIB_EXAMPLE_INTERFACES_TYPESUPPORT_CPP}")
    target_link_libraries(oracle_a_ros PRIVATE 
        ${LIB_ROSIDL_TYPESUPPORT_CPP} 
        ${LIB_EXAMPLE_INTERFACES_TYPESUPPORT_CPP}
    )
  else()
    message(WARNING "  Could not find explicit typesupport libraries. Linking may fail.")
  endif()
else()
  message(STATUS "ROS action deps not found: skipping oracle_a_ros target")
endif()
