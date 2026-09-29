cmake_minimum_required(VERSION 3.8)
project(pharmarobot_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md ARCHITECTURE.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/validate_slam_v1.py tools/slam_v1_readiness.py
  tools/slam_v1_trial_recorder.py tools/slam_v1_trial_stop.py
  tools/rviz_x11_diagnostic.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_slam_v1 test/test_slam_v1.py)
endif()

ament_package()
