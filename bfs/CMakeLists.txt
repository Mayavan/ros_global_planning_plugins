cmake_minimum_required(VERSION 2.8.3)
project(bfs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  base_local_planner
  costmap_2d
  nav_core
  pluginlib
  tf
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(bfs_planner src/bfs.cpp)
