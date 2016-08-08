cmake_minimum_required(VERSION 2.8.3)
project(diagnostics)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

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
)
