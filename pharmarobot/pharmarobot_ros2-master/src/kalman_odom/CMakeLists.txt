cmake_minimum_required(VERSION 3.5)
project(kalman_odom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(roboteq_ros2_driver REQUIRED)

# Add after find_package(...)
find_package(ament_cmake_python REQUIRED)

# Install Python nodes
install(PROGRAMS
  scripts/test_sim_input.py
  DESTINATION lib/${PROJECT_NAME}
)




add_executable(kalman_odometry_node src/kalman_odom_node.cpp)
ament_target_dependencies(kalman_odometry_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs 
  roboteq_ros2_driver)

#add_executable(test_sim_input src/test_sim_input.cpp)
#ament_target_dependencies(test_sim_input rclcpp geometry_msgs)

install(TARGETS
  kalman_odometry_node
 # test_sim_input
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
