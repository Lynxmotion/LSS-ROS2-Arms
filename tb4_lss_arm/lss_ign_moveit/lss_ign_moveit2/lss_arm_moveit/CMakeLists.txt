cmake_minimum_required(VERSION 3.10.2)
project(lss_arm_moveit)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(lss_arm_description REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
