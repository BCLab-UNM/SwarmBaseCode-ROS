cmake_minimum_required(VERSION 2.8.3)
project(rqt_rover_gui)

set(CMAKE_BUILD_TYPE Debug)

#Get version info from git and pass it into compiled code
execute_process(COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH)
string(STRIP "${GIT_BRANCH}" GIT_BRANCH)

execute_process(COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION)
string(STRIP "${GIT_VERSION}" GIT_VERSION)

execute_process(COMMAND date "+%Y-%m-%d-%H:%M"
    OUTPUT_VARIABLE BUILD_TIME)
string(STRIP ${BUILD_TIME} BUILD_TIME)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/Version.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/Version.cpp)

set(version_file "${CMAKE_CURRENT_BINARY_DIR}/Version.cpp")

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  rqt_gui
  rqt_gui_cpp
  cv_bridge
  image_transport
  geometry_msgs
  ublox_msgs
  ublox_serialization
)

find_package( Qt5 REQUIRED COMPONENTS Widgets Xml )

find_package( gazebo REQUIRED )
include_directories( ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

catkin_package( CATKIN_DEPENDS
  rqt_gui
  rqt_gui_cpp
  cv_bridge
  image_transport
  geometry_msgs
  swarmie_msgs
)

SET(rover_gui_plugin_RESOURCES resources/resources.qrc)

qt5_add_resources(rqt_rover_gui_resources_rcc ${rover_gui_plugin_RESOURCES})

qt5_wrap_cpp(
  rover_gui_plugin_MOCS
  src/rover_gui_plugin.h
  src/CameraFrame.h
  src/MapFrame.h
  src/USFrame.h
  src/GPSFrame.h
  src/IMUFrame.h
  src/JoystickGripperInterface.h
)

qt5_wrap_ui(
  rover_gui_plugin_UIS_H
  src/rover_gui_plugin.ui
)

include_directories(
  include
  src
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Xml_INCLUDE_DIRS}
)

link_directories(
  lib
)


add_library(
  rqt_rover_gui
  ${version_file}
  src/GazeboSimManager.cpp
  src/JoystickGripperInterface.cpp
  src/rover_gui_plugin.cpp
  src/CameraFrame.cpp
  src/MapFrame.cpp
  src/USFrame.cpp
  src/GPSFrame.cpp
  src/MapData.cpp
  src/IMUFrame.cpp
  src/BWTabWidget.cpp
  ${rover_gui_plugin_RESOURCES}
  ${rover_gui_plugin_MOCS}
  ${rover_gui_plugin_UIS_H}
  ${OpenCV_LIBS}
)

add_dependencies(
  rqt_rover_gui ${catkin_EXPORTED_TARGETS}
  ${ublox_msgs_EXPORTED_TARGETS}
)

target_link_libraries(
  rqt_rover_gui
  ${GAZEBO_libraries}
  ${catkin_LIBRARIES}
)

catkin_python_setup()

set(CMAKE_BUILD_TYPE Debug)
