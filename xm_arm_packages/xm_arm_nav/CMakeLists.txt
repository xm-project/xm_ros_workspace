cmake_minimum_required(VERSION 2.8.3)
project(xm_arm_nav)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
    roscpp
    xm_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include 
${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(xm_pick_or_place src/xm_pick_or_place.cpp)
target_link_libraries(xm_pick_or_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(xm_keyboard_teleop_arm src/xm_keyboard_teleop_arm.cpp)
target_link_libraries(xm_keyboard_teleop_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(xm_arm_ik_server src/xm_arm_ik_server.cpp)
target_link_libraries(xm_arm_ik_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})