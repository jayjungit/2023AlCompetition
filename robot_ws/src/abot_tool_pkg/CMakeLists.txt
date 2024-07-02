cmake_minimum_required(VERSION 3.0.2)
project(abot_tool_pkg)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  roslib
  std_msgs
  tf
  tf2_ros
  nav_msgs
  tf_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS 
  actionlib_msgs 
  geometry_msgs 
  roscpp 
  rospy 
  std_msgs 
  tf 
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_dist src/lidar_dist.cpp)

target_link_libraries(lidar_dist ${catkin_LIBRARIES})