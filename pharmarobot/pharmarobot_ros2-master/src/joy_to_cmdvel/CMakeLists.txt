cmake_minimum_required(VERSION 3.5)
project(joy_to_cmdvel)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create the executable
add_executable(joy_to_cmd_vel_node src/joy_to_cmd_vel.cpp)
ament_target_dependencies(joy_to_cmd_vel_node rclcpp sensor_msgs geometry_msgs)

# Install the executable
install(TARGETS
  joy_to_cmd_vel_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()