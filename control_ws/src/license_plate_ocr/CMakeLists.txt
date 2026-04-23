cmake_minimum_required(VERSION 3.0.2)
project(license_plate_ocr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  LicensePlateOCR.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs 
  sensor_msgs 
  message_runtime
)

catkin_install_python(PROGRAMS
  scripts/license_plate_server.py
  scripts/test_ocr_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)