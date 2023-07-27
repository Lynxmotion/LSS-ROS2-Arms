cmake_minimum_required(VERSION 3.5)
project(lss_arm)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(${PROJECT_NAME}_description REQUIRED)
find_package(${PROJECT_NAME}_moveit REQUIRED)

# Convert xacros of description and moveit packages to URDF, SDF and SRDF
get_filename_component(${PROJECT_NAME}_REPO_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR ${${PROJECT_NAME}_REPO_DIR}/${PROJECT_NAME}_description/scripts)
set(${PROJECT_NAME}_MOVEIT_CONFIG_SCRIPTS_DIR ${${PROJECT_NAME}_REPO_DIR}/${PROJECT_NAME}_moveit/scripts)
execute_process(
  COMMAND ${${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR}/xacro2urdf.bash
)
execute_process(
  COMMAND ${${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR}/xacro2sdf.bash
)
execute_process(
  COMMAND ${${PROJECT_NAME}_MOVEIT_CONFIG_SCRIPTS_DIR}/xacro2srdf.bash
)

# Setup the project
ament_package()
