cmake_minimum_required(VERSION 3.0.2)
project(scenario_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf
  visualization_msgs
  autoware_msgs
  std_msgs
  open_scenario_helper
  commonroad_helper
)

catkin_package(
  CATKIN_DEPENDS open_scenario_helper
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ../open_scenario_helper/include/ad-xolib/build/src/xosc
  ../open_scenario_helper/include/ad-xolib/build/src/xoeng
  ../open_scenario_helper/include/ad-xolib/build/src/xodr
  ../open_scenario_helper/include/ad-xolib/src/xosc
  ../open_scenario_helper/include/ad-xolib/src/xoeng
  ../open_scenario_helper/include/ad-xolib/src/xodr
  ../open_scenario_helper/include/ad-xolib/src

  ../commonroad_helper/include/commonroad
  ../commonroad_helper/include/opendrive
  ../commonroad_helper/include/pugi
)


add_executable(
  ${PROJECT_NAME}_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp
)

find_library(OSC_HELPER
NAMES open_scenario_helper
HINTS ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  proj
  ${OSC_HELPER}
)
