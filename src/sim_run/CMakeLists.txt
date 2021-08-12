cmake_minimum_required(VERSION 2.8.3)
project(sim_run)

find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs geometry_msgs visualization_msgs)
catkin_package()

catkin_install_python(PROGRAMS scripts/sim_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})