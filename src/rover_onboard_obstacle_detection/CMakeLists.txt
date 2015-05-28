cmake_minimum_required(VERSION 2.8.3)
project(rover_onboard_obstacle_detection)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_onboard_obstacle_detection
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  obstacle src/obstacle.cpp
)

target_link_libraries(obstacle
  ${catkin_LIBRARIES}
)


