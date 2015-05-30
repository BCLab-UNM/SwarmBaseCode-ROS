cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_path_planning)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
)

add_executable(
  path src/path.cpp
)

add_executable(
  path_tester src/path_tester.cpp
)

target_link_libraries(
  path
  ${catkin_LIBRARIES}
)

target_link_libraries(
  path_tester
  ${catkin_LIBRARIES}
)

