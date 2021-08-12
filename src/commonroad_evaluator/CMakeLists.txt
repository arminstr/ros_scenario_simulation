cmake_minimum_required(VERSION 2.8.3)
project(commonroad_evaluator)

find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs geometry_msgs)
catkin_package()

catkin_install_python(PROGRAMS scripts/evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})