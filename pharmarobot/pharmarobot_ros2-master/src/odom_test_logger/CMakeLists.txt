cmake_minimum_required(VERSION 3.8)
project(odom_test_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(roboteq_ros2_driver REQUIRED)

add_executable(odom_test_logger_node src/odom_test_logger.cpp)

ament_target_dependencies(
  odom_test_logger_node
  rclcpp
  geometry_msgs
  nav_msgs
  roboteq_ros2_driver
)

install(TARGETS
  odom_test_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
