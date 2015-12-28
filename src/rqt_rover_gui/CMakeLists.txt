cmake_minimum_required(VERSION 2.8.3)
project(rqt_rover_gui)

set(CMAKE_BUILD_TYPE Debug)

execute_process(COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION)
string(STRIP ${GIT_VERSION} GIT_VERSION)

execute_process(COMMAND date "+%Y-%m-%d-%H:%M"
    OUTPUT_VARIABLE BUILD_TIME)
string(STRIP ${BUILD_TIME} BUILD_TIME)

set_source_files_properties(src/rover_gui_plugin.cpp PROPERTIES COMPILE_DEFINITIONS "GIT_VERSION=\"${GIT_VERSION}\";BUILD_TIME=\"${BUILD_TIME}\"")

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  rqt_gui
  rqt_gui_cpp
  cv_bridge
  image_transport
)

find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
  QtXml
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp cv_bridge image_transport
)

SET(rover_gui_plugin_RESOURCES resources/resources.qrc)

qt4_add_resources(rqt_rover_gui_resources_rcc ${rover_gui_plugin_RESOURCES})

qt4_wrap_cpp(
  rover_gui_plugin_MOCS
  src/rover_gui_plugin.h
  src/CameraFrame.h
  src/MapFrame.h
  src/USFrame.h
  src/GPSFrame.h
  src/IMUFrame.h
  #src/IMUWidget.h
)

qt4_wrap_ui(
  rover_gui_plugin_UIS_H
  src/rover_gui_plugin.ui
)

include(
  ${QT_USE_FILE}
)

include_directories(
  src
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(
  rqt_rover_gui
  src/GazeboSimManager.cpp
  src/rover_gui_plugin.cpp
  src/CameraFrame.cpp
  src/MapFrame.cpp
  src/USFrame.cpp
  src/GPSFrame.cpp
  #src/IMUWidget.cpp
  src/IMUFrame.cpp
  src/BWTabWidget.cpp
  ${rover_gui_plugin_RESOURCES}
  ${rover_gui_plugin_MOCS}
  ${rover_gui_plugin_UIS_H}
  ${OpenCV_LIBS}
)

target_link_libraries(
  rqt_rover_gui
  ${GAZEBO_libraries}
  ${catkin_LIBRARIES}
)

catkin_python_setup()

set(CMAKE_BUILD_TYPE Debug)
