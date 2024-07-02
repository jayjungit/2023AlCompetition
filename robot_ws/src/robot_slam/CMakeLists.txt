cmake_minimum_required(VERSION 2.8.3)
project(robot_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  tf_conversions
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_slam
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
