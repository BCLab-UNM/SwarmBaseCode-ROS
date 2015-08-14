cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_gps)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  include
)

add_executable(
  gps src/gps.cpp src/usbSerial.cpp
)

target_link_libraries(
  gps
  ${catkin_LIBRARIES}
)

