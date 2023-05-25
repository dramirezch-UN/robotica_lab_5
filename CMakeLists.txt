cmake_minimum_required(VERSION 2.8.3)
project(lab_5)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  code/python/draw_workspace.py
  code/python/draw.py
  code/python/jointPub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
