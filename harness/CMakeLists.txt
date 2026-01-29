cmake_minimum_required(VERSION 3.16)
project(ros2_semantic_oracle_harness CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(oracle_a src/oracle_a.cpp)
target_include_directories(oracle_a PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

add_executable(audit_trace src/audit_trace.cpp)
target_include_directories(audit_trace PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
