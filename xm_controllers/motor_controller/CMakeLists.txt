cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  xm_msgs
  realtime_tools
  roscpp
  handsfree_hw
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/motor_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
