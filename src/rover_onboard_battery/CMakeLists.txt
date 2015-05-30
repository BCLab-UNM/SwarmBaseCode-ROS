cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_battery)

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
  battery src/battery.cpp
)

target_link_libraries(
  battery
  ${catkin_LIBRARIES}
)

