cmake_minimum_required(VERSION 3.0.2)
project(commonroad_helper)

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
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)
set(PROJECT_SOURCE_LIST
  # ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CommonRoad.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/pugixml.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/CommonRoadParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/GeometryParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/GeoReferenceParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/JunctionParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/LaneParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/OpenDriveParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/ProfilesParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/RoadLinkParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/TrafficGroupParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/TrafficSignalsParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/TrafficSignParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/CenterLine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/Geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/GeometryGenerator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/LaneUtils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenDrive.cpp
)


add_library(
  ${PROJECT_NAME}
  ${PROJECT_SOURCE_LIST}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
