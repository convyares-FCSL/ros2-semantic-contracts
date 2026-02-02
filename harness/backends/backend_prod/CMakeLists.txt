cmake_minimum_required(VERSION 3.8)
project(backend_prod)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(backend_prod src/main.cpp)
target_include_directories(backend_prod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(backend_prod PUBLIC cxx_std_20)
ament_target_dependencies(backend_prod rclcpp)

install(TARGETS backend_prod
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
