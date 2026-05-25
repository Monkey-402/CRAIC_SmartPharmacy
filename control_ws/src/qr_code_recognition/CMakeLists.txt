cmake_minimum_required(VERSION 3.0.2)
project(qr_code_recognition)

find_package(catkin REQUIRED COMPONENTS
  move_nav
  rospy
)

catkin_package(
  CATKIN_DEPENDS move_nav rospy
)

catkin_install_python(PROGRAMS
  qr_decoder.py
  qr_node.py
  qr_parser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
