cmake_minimum_required(VERSION 3.0.2)
project(text_recognition)

find_package(catkin REQUIRED COMPONENTS
  move_nav
  rospy
)

catkin_package(
  CATKIN_DEPENDS move_nav rospy
)

catkin_install_python(PROGRAMS
  scripts/ocr_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
