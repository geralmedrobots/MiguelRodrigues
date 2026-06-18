cmake_minimum_required(VERSION 3.8)
project(command_arbiter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(command_arbiter_node src/command_arbiter.cpp)
target_compile_features(command_arbiter_node PUBLIC cxx_std_17)
ament_target_dependencies(command_arbiter_node geometry_msgs rclcpp)

install(TARGETS command_arbiter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
