cmake_minimum_required(VERSION 2.8.3)
project(behaviours)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
  apriltags_ros
  swarmie_msgs
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs swarmie_msgs roscpp sensor_msgs std_msgs random_numbers tf apriltags_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(swarmie_interface
  src/SwarmieSensors.cpp
  src/SwarmieInterface.cpp
)

add_library(behavior
    src/BehaviorManager.cpp
    src/ObstacleBehavior.cpp
    src/StraightLineBehavior.cpp
    src/AvoidNest.cpp
    src/AlignToCube.cpp
    src/PickUpCube.cpp    
)

add_executable(
  behaviours
  src/Behavior.cpp
#  src/Tag.cpp
#  src/ObstacleController.cpp 
#  src/PickUpController.cpp
#  src/DropOffController.cpp
#  src/SearchController.cpp
#  src/ROSAdapter.cpp
#  src/PID.cpp
#  src/DriveController.cpp
#  src/RangeController.cpp
#  src/LogicController.cpp
#  src/ManualWaypointController.cpp
)

add_dependencies(behaviours ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  behaviours
  behavior
  swarmie_interface
  ${catkin_LIBRARIES}
)

catkin_add_gtest(behaviour_test
  test/test_util.cpp
  test/test.cpp
  test/obstacle_test.cpp)

target_link_libraries(behaviour_test behavior swarmie_interface ${catkin_LIBRARIES})
