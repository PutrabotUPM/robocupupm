cmake_minimum_required(VERSION 3.0.2)
project(dijkstra_planner)

## Use catkin's CMake macros.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

## Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add source files for the library
add_library(dijkstra_planner src/dijkstra_planner.cpp)  # Make sure this is correct!

## Declare a C++ executable
add_executable(dijkstra_node src/dijkstra_node.cpp)

## Link libraries
target_link_libraries(dijkstra_node
  ${catkin_LIBRARIES}
  dijkstra_planner
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dijkstra_planner
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs
)

