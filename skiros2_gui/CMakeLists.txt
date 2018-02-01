cmake_minimum_required(VERSION 2.8.3)
project(skiros2_gui) 

find_package(catkin REQUIRED COMPONENTS
  rospy
  skiros2_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy skiros2_msgs
  DEPENDS
)
