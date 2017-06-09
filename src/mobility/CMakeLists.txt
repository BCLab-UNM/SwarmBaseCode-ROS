cmake_minimum_required(VERSION 2.8.3)
project(mobility)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mobility
  src/ObstacleController.cpp 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
  src/mobility.cpp
  src/PID.cpp
)

add_dependencies(mobility ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)

