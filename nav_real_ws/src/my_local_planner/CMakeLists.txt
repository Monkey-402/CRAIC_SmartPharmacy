cmake_minimum_required(VERSION 2.8.3)
project(my_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  nav_core
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  base_local_planner
  angles
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_local_planner
  CATKIN_DEPENDS 
    roscpp 
    costmap_2d 
    nav_core 
    pluginlib 
    tf2_ros 
    tf2_geometry_msgs
    dynamic_reconfigure 
    base_local_planner
    angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(my_local_planner
  src/my_local_planner.cpp
)

target_link_libraries(my_local_planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS my_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES my_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)