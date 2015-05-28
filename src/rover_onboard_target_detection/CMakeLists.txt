cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_target_detection)
set(CMAKE_C_FLAGS "-std=c99" )
set (CMAKE_CXX_FLAGS "-fpermissive")
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
  INCLUDE_DIRS include
  LIBRARIES rover_onboard_workspace
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(include)
include_directories(include/AprilTags)
include_directories(${catkin_INCLUDE_DIRS}
)

link_directories(~/rover_workspace/src/rover_onboard_target_detection/lib)

add_executable(
  camera src/camera.cpp src/usbCamera.cpp
)
add_executable(
  target src/target.cpp
)

add_dependencies(
	target rover_onboard_target_detection_generate_messages_cpp
)

target_link_libraries(camera
  ${catkin_LIBRARIES}
)

target_link_libraries(target
  libapriltag.a 
  #libapriltags.a
  ${catkin_LIBRARIES}
)


