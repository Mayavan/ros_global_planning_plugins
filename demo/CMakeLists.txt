cmake_minimum_required(VERSION 2.8.3)
project(demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  move_base_msgs 
  actionlib
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(demo src/main.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${OpenCV_LIBS})


