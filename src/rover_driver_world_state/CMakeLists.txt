cmake_minimum_required(VERSION 2.8.3)
project(rover_driver_world_state)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
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
  updateMap.srv
  trackPheromone.srv
)

generate_messages(
   DEPENDENCIES
   #geometry_msgs
   #sensor_msgs
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES rover_driver_workspace
  LIBRARIES rover_onboard_workspace
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime
  DEPENDS system_lib
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}  
)
#include_directories(~/rover_onboard_workspace/devel/include)

link_directories(
   ~/rover_workspace/src/rover_driver_world_state/lib
)

add_executable(
  world_state src/world_state.cpp
)

add_dependencies(
	world_state rover_driver_world_state_generate_messages_cpp	
)

target_link_libraries(world_state
  ${catkin_LIBRARIES}
)


