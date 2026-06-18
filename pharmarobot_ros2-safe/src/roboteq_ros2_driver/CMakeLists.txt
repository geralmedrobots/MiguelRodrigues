cmake_minimum_required(VERSION 3.5)
project(roboteq_ros2_driver)

# ---------------------------------------------------------------------------
# Build settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS thread)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# ---------------------------------------------------------------------------
# Message generation
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelTicks.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ---------------------------------------------------------------------------
# Optional helper library
# ---------------------------------------------------------------------------
add_library(roboteq_protocol
  src/roboteq_protocol.cpp
)

target_include_directories(roboteq_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(roboteq_odom_tf
  src/odom_tf.cpp
)

target_include_directories(roboteq_odom_tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roboteq_odom_tf
  builtin_interfaces
  geometry_msgs
  tf2
)

add_library(roboteq_odom_covariance
  src/odom_covariance.cpp
)

target_include_directories(roboteq_odom_covariance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(roboteq_odom_twist
  src/odom_twist.cpp
)

target_include_directories(roboteq_odom_twist PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(roboteq_command_scaling
  src/command_scaling.cpp
)

target_include_directories(roboteq_command_scaling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(differential_drive_kinematics
  src/differential_drive_kinematics/differential_drive_kinematics.cpp
)

target_include_directories(differential_drive_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/differential_drive_kinematics>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(differential_drive_kinematics
  rclcpp
)

# ---------------------------------------------------------------------------
# Main node executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/driver_dev.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  differential_drive_kinematics
  roboteq_command_scaling
  roboteq_odom_covariance
  roboteq_odom_tf
  roboteq_odom_twist
  roboteq_protocol
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  serial
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link generated WheelTicks message interface
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Export dependencies
# ---------------------------------------------------------------------------
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial
  rosidl_default_runtime
)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
install(TARGETS
  roboteq_protocol
  roboteq_command_scaling
  roboteq_odom_covariance
  roboteq_odom_tf
  roboteq_odom_twist
  differential_drive_kinematics
  ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_roboteq_protocol
    test/test_roboteq_protocol.cpp
  )

  target_link_libraries(test_roboteq_protocol
    roboteq_protocol
  )

  ament_add_gtest(test_odom_tf
    test/test_odom_tf.cpp
  )

  target_link_libraries(test_odom_tf
    roboteq_odom_tf
  )

  ament_add_gtest(test_odom_covariance
    test/test_odom_covariance.cpp
  )

  target_link_libraries(test_odom_covariance
    roboteq_odom_covariance
  )

  ament_add_gtest(test_odom_twist
    test/test_odom_twist.cpp
  )

  target_link_libraries(test_odom_twist
    roboteq_odom_twist
  )

  ament_add_gtest(test_command_scaling
    test/test_command_scaling.cpp
  )

  target_link_libraries(test_command_scaling
    roboteq_command_scaling
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
