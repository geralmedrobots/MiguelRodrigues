cmake_minimum_required(VERSION 3.5)
project(roboteq_ros2_driver)

# ---------------------------------------------------------------------------
# Build settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(nav_msgs             REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(tf2                  REQUIRED)
find_package(tf2_ros              REQUIRED)
find_package(tf2_geometry_msgs    REQUIRED)
find_package(serial               REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS thread)
find_package(rosidl_default_generators REQUIRED)     # ✨ message code‑gen
find_package(rosidl_default_runtime    REQUIRED)     # ✨ message linkage

# ---------------------------------------------------------------------------
# Message generation  (⇦ happens *before* any C++ is compiled)
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelTicks.msg"          # keep the original file name; header will be wheel_ticks.hpp
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ---------------------------------------------------------------------------
# Optional helper library
# ---------------------------------------------------------------------------
add_library(differential_drive_kinematics
  src/differential_drive_kinematics/differential_drive_kinematics.cpp
)
target_include_directories(differential_drive_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/differential_drive_kinematics>
)
ament_target_dependencies(differential_drive_kinematics rclcpp)
ament_export_dependencies(rclcpp)  # Add any actual deps
# ---------------------------------------------------------------------------
# Main node executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/driver_dev.cpp)

target_link_libraries(${PROJECT_NAME}_node
  differential_drive_kinematics
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  serial
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 🔑 Tie the node to the generated message interfaces
#rosidl_target_interfaces(${PROJECT_NAME}_node
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
install(TARGETS
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

# ---------------------------------------------------------------------------
# Tests (optional)
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
