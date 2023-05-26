cmake_minimum_required(VERSION 3.0.2)
project(lab_5)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lab_5.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
