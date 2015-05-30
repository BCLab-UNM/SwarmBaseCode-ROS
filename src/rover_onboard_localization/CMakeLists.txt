cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
)

add_executable(
  localization src/localization.cpp
)

target_link_libraries(
  localization
  ${catkin_LIBRARIES}
)

