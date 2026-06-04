cmake_minimum_required(VERSION 3.0.2)
project(board2_paddle_ocr)

find_package(catkin REQUIRED COMPONENTS
  move_nav
  rospy
)

catkin_package(
  CATKIN_DEPENDS move_nav rospy
)

catkin_install_python(PROGRAMS
  scripts/ocr_paddle_service.py
  scripts/paddle_ocr_client.py
  scripts/decode_board2_paddle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  run_paddle_ocr_server.sh
  setup_paddle_conda.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY paddle_ocr/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paddle_ocr
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
