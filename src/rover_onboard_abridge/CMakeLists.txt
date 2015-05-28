cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_abridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES abridge
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)

add_executable(abridge src/abridge.cpp src/usbSerial.cpp)

 target_link_libraries(abridge
   ${catkin_LIBRARIES}
 )

