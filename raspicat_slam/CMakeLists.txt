cmake_minimum_required(VERSION 3.8)
project(raspicat_slam)
find_package(ament_cmake_auto REQUIRED)
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
