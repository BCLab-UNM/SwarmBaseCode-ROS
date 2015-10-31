cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  random_numbers
)

add_message_files(
  FILES
  ATag.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs message_runtime random_numbers
)

add_executable(
  mobility src/mobility.cpp
)

add_dependencies(
  mobility rover_onboard_target_detection_generate_messages_cpp
)

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)

