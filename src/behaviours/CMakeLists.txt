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
  mavros_msgs
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs swarmie_msgs roscpp sensor_msgs std_msgs random_numbers tf apriltags_ros mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(core_interface
  src/Heading.cpp
  src/Velocity.cpp
  src/Point.cpp
  src/Action.cpp
)

add_library(swarmie_interface
  src/SwarmieSensors.cpp
  src/SwarmieInterface.cpp
)

add_library(utilities
   src/ROSTimer.cpp
   src/PID.cpp
)

add_library(behavior
    src/BehaviorManager.cpp
    src/ObstacleBehavior.cpp
    src/StraightLineBehavior.cpp
    src/AvoidNest.cpp
    src/AlignToCube.cpp
    src/ApproachCube.cpp
    src/PickUpCube.cpp
    src/DropOffCube.cpp
)

add_executable(
  behaviours
  src/Behavior.cpp
)

add_executable(
  JUAV_behaviours
  src/JUAV_Behavior.cpp
)

add_dependencies(behaviours ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  behaviours
  behavior
  core_interface
  swarmie_interface
  utilities
  ${catkin_LIBRARIES}
)

target_link_libraries(
  JUAV_behaviours
  behavior
  swarmie_interface
  utilities
  ${catkin_LIBRARIES}
)

catkin_add_gmock(behaviour_test
  test/test_util.cpp
  test/test.cpp
  test/obstacle_test.cpp
  test/avoid_nest_test.cpp
  test/align_to_cube_test.cpp
  test/approach_cube_test.cpp
  test/straight_line_test.cpp
  test/pickup_test.cpp
  test/velocity_test.cpp
  test/point_test.cpp
  test/waypoint_test.cpp
  test/action_util.cpp
  test/heading_test.cpp
)

target_link_libraries(behaviour_test behavior core_interface swarmie_interface utilities ${catkin_LIBRARIES})
