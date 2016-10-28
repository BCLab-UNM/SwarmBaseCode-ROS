cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins)

set(CMAKE_CXX_FLAGS "-std=c++0x")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} src src/GripperPlugin ${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} src/SetupWorld.cpp)

add_library(${PROJECT_NAME}_gripper 
  src/GripperPlugin/GripperPlugin.cpp
  src/GripperPlugin/PIDController.cpp 
  src/GripperPlugin/GripperManager.cpp)

add_library(${PROJECT_NAME}_score
  src/ScorePlugin/ScorePlugin.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

