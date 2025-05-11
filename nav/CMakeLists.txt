cmake_minimum_required(VERSION 3.0.2)
project(nav)

find_package(catkin REQUIRED COMPONENTS
  amcl
  gmapping
  map_server
  move_base
  
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  
  message_generation
)

catkin_package()

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/dijkstra_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare the service files to be built
add_service_files(
  FILES
  Dijkstra.srv
)


## Generate added messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

## Include message_runtime in catkin_package
catkin_package(
  CATKIN_DEPENDS message_runtime ...
)
