cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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
  trackPheromone.srv
)

# generate_messages()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ~/rover_workspace/devel/include
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES abridge
  LIBRARIES rover_onboard_workspace
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs message_runtime
  DEPENDS system_lib
)
add_executable(
   mobility src/mobility.cpp
)

add_dependencies(
	mobility rover_onboard_target_detection_generate_messages_cpp
)

 target_link_libraries(mobility
   ${catkin_LIBRARIES}
 )

