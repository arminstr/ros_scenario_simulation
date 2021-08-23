cmake_minimum_required(VERSION 3.0.2)
project(tf_broadcaster)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  # /home/path/ad-xolib/build/src/xosc
)
 add_executable(${PROJECT_NAME}_node src/tf_broadcaster_node.cpp)


# find_library(XOSC_LIBRARY
# NAMES xosc libxosc 
# HINTS "/home/path/ad-xolib/build/src/xosc"
# )

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
  #  ${XOSC_LIBRARY}
 )
