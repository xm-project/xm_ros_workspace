cmake_minimum_required(VERSION 2.8.3)
project(xm_smach)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  rospy
  smach
  smach_ros
  std_msgs
  std_srvs
  tf
  xm_msgs
  roscpp
)
catkin_python_setup()
catkin_package()