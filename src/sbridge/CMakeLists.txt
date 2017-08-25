cmake_minimum_required(VERSION 2.8.3)
project(sbridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(
  sbridge src/main.cpp src/sbridge.cpp
)

target_link_libraries(
  sbridge
  ${catkin_LIBRARIES}
)

