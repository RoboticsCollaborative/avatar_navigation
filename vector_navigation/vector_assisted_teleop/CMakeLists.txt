cmake_minimum_required(VERSION 2.8.3)
project(vector_assisted_teleop)

find_package(cmake_modules REQUIRED)

set(THIS_PACKAGE_ROS_DEPS
  tf2_ros
  costmap_2d
  roscpp
  roslib
  geometry_msgs
  move_base_msgs
  actionlib
  message_filters
  base_local_planner
  angles
  pluginlib
  sensor_msgs
  filters
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS})
message("Assisted telop catkin_LIBRARY_DIRS: " ${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
)

add_executable(vector_assisted_teleop src/vector_assisted_teleop.cpp)
target_link_libraries(vector_assisted_teleop trajectory_planner_ros ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

install(TARGETS vector_assisted_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


