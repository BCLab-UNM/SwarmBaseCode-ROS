cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers
)

add_executable(
  mobility src/mobility.cpp
)

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)

