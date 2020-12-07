cmake_minimum_required(VERSION 3.0.2)
project(agv_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


