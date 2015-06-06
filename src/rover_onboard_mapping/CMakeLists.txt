cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_mapping)

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
  updateMap.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime
)

add_executable(
  mapping src/mapping.cpp
)

add_dependencies(
  mapping rover_onboard_target_detection_generate_messages_cpp
)

target_link_libraries(
  mapping
  ${catkin_LIBRARIES}
)


