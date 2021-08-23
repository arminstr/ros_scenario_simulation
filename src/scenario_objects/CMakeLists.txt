cmake_minimum_required(VERSION 3.0.2)
project(scenario_objects)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf2
  autoware_msgs
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

## Specify additional locations of header files
## Your package locations should be listed before other locations
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

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  proj
)
