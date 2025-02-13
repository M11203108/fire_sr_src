################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(wheeltec_robot_urdf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

################################################################################
# Install
################################################################################
install(DIRECTORY meshes rviz urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(ros2_control)
ament_export_dependencies(controller_manager)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(gazebo_ros2_control)

ament_package()
