cmake_minimum_required(VERSION 3.5)
project(teleop_pharma)

# Find required packages
find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()