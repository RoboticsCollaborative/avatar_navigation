cmake_minimum_required(VERSION 3.0.2)
project(crisp_ui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosbridge_server
  roscpp
  rospy
  std_msgs
  geometry_msgs
  crisp_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crisp_common
  CATKIN_DEPENDS rosbridge_server message_runtime roscpp rospy std_msgs crisp_common
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

