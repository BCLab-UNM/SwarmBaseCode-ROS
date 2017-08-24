cmake_minimum_required(VERSION 2.8.3)
project(diagnostics)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
)

catkin_package(
  DEPENDS
  roscpp
  std_msgs
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(
  diagnostics 
  src/driver.cpp
  src/Diagnostics.cpp
  src/WirelessDiags.cpp
)

add_dependencies(diagnostics ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  diagnostics
  usb
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
