cmake_minimum_required(VERSION 3.5)
project(roboteq_ros2_driver)

option(ROBOTEQ_BUILD_HARDWARE_VALIDATION
  "Build fixed-mode Roboteq hardware validation harnesses" OFF)

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
find_package(diagnostic_msgs REQUIRED)
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

add_library(roboteq_command_watchdog
  src/command_watchdog.cpp
)

target_include_directories(roboteq_command_watchdog PUBLIC
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

add_library(roboteq_command_conversion
  src/roboteq_command_conversion.cpp
)

target_include_directories(roboteq_command_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(roboteq_configuration
  src/roboteq_configuration.cpp
)

add_library(roboteq_diagnostics
  src/roboteq_diagnostics.cpp
)

target_include_directories(roboteq_diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roboteq_diagnostics
  diagnostic_msgs
  rclcpp
  serial
)

target_include_directories(roboteq_configuration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(roboteq_driver_parameter_validation
  src/driver_parameter_validation.cpp
)

target_include_directories(roboteq_driver_parameter_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(roboteq_serial_transport
  src/roboteq_serial_transport.cpp
)

target_include_directories(roboteq_serial_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roboteq_serial_transport
  serial
)

add_library(roboteq_serial_worker
  src/roboteq_serial_worker.cpp
)

add_library(roboteq_telemetry
  src/roboteq_telemetry.cpp
)

target_include_directories(roboteq_telemetry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(roboteq_serial_worker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(roboteq_serial_worker
  roboteq_command_scaling
  roboteq_configuration
  roboteq_protocol
  roboteq_serial_transport
  roboteq_telemetry
)

if(ROBOTEQ_BUILD_HARDWARE_VALIDATION)
  add_executable(roboteq_phase5b_stop_latency_validation
    tools/phase5b/roboteq_phase5b_stop_latency_validation.cpp
  )
  target_link_libraries(roboteq_phase5b_stop_latency_validation
    roboteq_serial_transport
    roboteq_serial_worker
  )
  ament_target_dependencies(roboteq_phase5b_stop_latency_validation serial)
  install(TARGETS roboteq_phase5b_stop_latency_validation
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  add_executable(roboteq_phase5b_ack_characterization
    tools/phase5b/roboteq_phase5b_ack_characterization.cpp
  )
  target_include_directories(roboteq_phase5b_ack_characterization PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(roboteq_phase5b_ack_characterization serial)
  install(TARGETS roboteq_phase5b_ack_characterization
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_target_dependencies(roboteq_serial_worker
  serial
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

add_library(roboteq_odometry
  src/roboteq_odometry.cpp
)

target_include_directories(roboteq_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/differential_drive_kinematics>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(roboteq_odometry
  differential_drive_kinematics
  roboteq_odom_twist
)

# ---------------------------------------------------------------------------
# Main node executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/driver_dev.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  differential_drive_kinematics
  roboteq_command_watchdog
  roboteq_command_conversion
  roboteq_command_scaling
  roboteq_configuration
  roboteq_driver_parameter_validation
  roboteq_diagnostics
  roboteq_odometry
  roboteq_odom_covariance
  roboteq_odom_tf
  roboteq_odom_twist
  roboteq_protocol
  roboteq_serial_transport
  roboteq_serial_worker
  roboteq_telemetry
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  diagnostic_msgs
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

# Link generated WheelTicks message interface (Foxy-compatible API)
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
  roboteq_command_watchdog
  roboteq_command_conversion
  roboteq_command_scaling
  roboteq_configuration
  roboteq_diagnostics
  roboteq_driver_parameter_validation
  roboteq_odometry
  roboteq_odom_covariance
  roboteq_odom_tf
  roboteq_odom_twist
  roboteq_serial_transport
  roboteq_serial_worker
  roboteq_telemetry
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

install(FILES
  src/differential_drive_kinematics/differential_drive_kinematics.hpp
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  if(ROBOTEQ_BUILD_HARDWARE_VALIDATION)
    add_test(NAME test_phase5b_harness_rejects_nonfixed_port
      COMMAND roboteq_phase5b_stop_latency_validation
        --port /tmp/not-roboteq --baud 115200 --operator offline-test
        --output /tmp/must-not-be-created.jsonl normal
        --query FF --deadline-ms 100 --attempts 1 --phase-plan after-write)
    set_tests_properties(test_phase5b_harness_rejects_nonfixed_port PROPERTIES WILL_FAIL TRUE)

    add_test(NAME test_phase5b_ack_characterization_rejects_nonfixed_port
      COMMAND roboteq_phase5b_ack_characterization
        --port /tmp/not-roboteq --baud 115200 --output /tmp/must-not-be-created.jsonl
        --mode h2 --attempts 1)
    set_tests_properties(
      test_phase5b_ack_characterization_rejects_nonfixed_port PROPERTIES WILL_FAIL TRUE)
  endif()

  ament_add_gtest(test_roboteq_protocol
    test/test_roboteq_protocol.cpp
  )

  target_link_libraries(test_roboteq_protocol
    roboteq_protocol
  )

  ament_add_gtest(test_command_watchdog
    test/test_command_watchdog.cpp
  )

  target_link_libraries(test_command_watchdog
    roboteq_command_watchdog
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

  ament_add_gtest(test_command_conversion
    test/test_command_conversion.cpp
  )

  target_link_libraries(test_command_conversion
    roboteq_command_conversion
  )

  ament_add_gtest(test_roboteq_configuration
    test/test_roboteq_configuration.cpp
  )

  target_link_libraries(test_roboteq_configuration
    roboteq_configuration
  )

  ament_add_gtest(test_roboteq_diagnostics
    test/test_roboteq_diagnostics.cpp
  )
  target_link_libraries(test_roboteq_diagnostics
    roboteq_diagnostics
    roboteq_driver_parameter_validation
  )

  ament_add_gtest(test_roboteq_odometry
    test/test_roboteq_odometry.cpp
  )

  target_link_libraries(test_roboteq_odometry
    roboteq_odometry
  )

  ament_add_gtest(test_driver_parameter_validation
    test/test_driver_parameter_validation.cpp
  )

  target_link_libraries(test_driver_parameter_validation
    roboteq_driver_parameter_validation
  )

  ament_add_gtest(test_serial_worker
    test/test_serial_worker.cpp
  )

  target_link_libraries(test_serial_worker
    roboteq_serial_worker
  )

  ament_target_dependencies(test_serial_worker
    serial
  )

  ament_add_gtest(test_roboteq_telemetry
    test/test_roboteq_telemetry.cpp
  )
  target_link_libraries(test_roboteq_telemetry
    roboteq_telemetry
  )

  ament_add_gtest(test_serial_transport
    test/test_serial_transport.cpp
  )

  target_link_libraries(test_serial_transport
    roboteq_serial_transport
  )

  ament_target_dependencies(test_serial_transport
    serial
  )

  ament_add_gtest(test_phase5b_harness
    test/test_phase5b_harness.cpp
  )
  target_include_directories(test_phase5b_harness PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_phase5b_harness
    serial
  )

  ament_add_gtest(test_phase5b_ack_characterization
    test/test_phase5b_ack_characterization.cpp
  )
  target_include_directories(test_phase5b_ack_characterization PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_phase5b_ack_characterization
    serial
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
