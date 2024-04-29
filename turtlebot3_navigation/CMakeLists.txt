################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_navigation)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  geometry_msgs
)

find_package(yaml-cpp REQUIRED)
include_directories(HEADERS_EXECUTABLE
    ${YAML_INCLUDE_DIRS}
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories( 
  include 
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
add_executable(yaml_rw src/yaml_rw.cpp)
target_link_libraries(yaml_rw 
  ${catkin_LIBRARIES}
)

add_executable(save_clicked_point src/save_clicked_point.cpp)
target_link_libraries(save_clicked_point 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)