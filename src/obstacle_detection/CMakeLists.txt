cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_filters
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs message_filters
)

add_executable(
  obstacle src/obstacle.cpp
)

target_link_libraries(
  obstacle
  ${catkin_LIBRARIES}
)


