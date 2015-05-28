cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_battery)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES battery
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(include)

## Declare a cpp executable
add_executable(battery src/battery.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(battery
   ${catkin_LIBRARIES}
 )

