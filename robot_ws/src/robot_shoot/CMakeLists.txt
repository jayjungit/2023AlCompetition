cmake_minimum_required(VERSION 3.0.2)
project(robot_shoot)

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

add_executable(nav_shoot src/main.cpp src/nav_shoot.cpp)
target_link_libraries(nav_shoot ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libyaml-cpp.a)

add_executable(nav_shoot_debug src/shoot_debug.cpp src/nav_shoot.cpp)
target_link_libraries(nav_shoot_debug ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libyaml-cpp.a)
