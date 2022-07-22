cmake_minimum_required(VERSION 3.0.2)
project(raspicat_navigation)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  raspimouse_msgs
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lightsensors_to_laserscan
  src/lightsensors_to_laserscan.cpp
)
add_dependencies(lightsensors_to_laserscan
  ${TARGET_NAME} raspimouse_msgs_generate_messages_cpp
)
add_executable(lightsensors_to_laserscan_node
  src/lightsensors_to_laserscan_node.cpp
)
target_link_libraries(
  lightsensors_to_laserscan_node
  lightsensors_to_laserscan
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()