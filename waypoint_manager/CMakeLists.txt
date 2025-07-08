################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(waypoint_manager)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS 
  actionlib
  move_base_msgs
  roscpp 
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
)

find_package(yaml-cpp REQUIRED)


include_directories(HEADERS_EXECUTABLE
    ${YAML_INCLUDE_DIRS}
)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
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
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf
)

include_directories( 
  include 
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Test
################################################################################
add_executable(waypoint_navigation src/waypoint_navigation.cpp)
target_link_libraries(waypoint_navigation 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(save_data 
  src/save_move_path.cpp 
  src/save_object_local.cpp 
  src/save_polygon_local.cpp
  # ${MOC_FILES}
  src/main.cpp)
target_link_libraries(save_data 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)


add_dependencies(save_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})