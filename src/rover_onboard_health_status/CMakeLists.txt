cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_health_status)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable(
  health_status src/health_status.cpp
)

target_link_libraries(
  health_status
  ${catkin_LIBRARIES}
)

