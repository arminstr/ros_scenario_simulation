cmake_minimum_required(VERSION 3.0.2)
project(scenario_goal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  visualization_msgs
  open_scenario_helper
  commonroad_helper
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES commonroad
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
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

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  proj
)
