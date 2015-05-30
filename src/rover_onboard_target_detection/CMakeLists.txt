cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_target_detection)

find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ATag.msg
)

add_service_files(
  FILES
  harvest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime
)

include_directories(
  include
)

link_directories(
  lib
)

add_executable(
  camera src/camera.cpp src/usbCamera.cpp
)

add_executable(
  target src/target.cpp
)

add_dependencies(
  target rover_onboard_target_detection_generate_messages_cpp
)

target_link_libraries(
  camera
  ${catkin_LIBRARIES}
)

target_link_libraries(
  target
  libapriltag.a 
  ${catkin_LIBRARIES}
)


