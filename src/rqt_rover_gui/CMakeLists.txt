cmake_minimum_required(VERSION 2.8.3)
project(rqt_rover_gui)

#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  rqt_gui
  rqt_gui_cpp
  cv_bridge
  image_transport
)

find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
)

catkin_package(
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp cv_bridge image_transport
)

qt4_wrap_cpp(
  rover_gui_plugin_MOCS
  src/rover_gui_plugin.h
  src/CameraFrame.h
  src/MapFrame.h
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
  src/rover_gui_plugin.cpp
  src/CameraFrame.cpp
  src/MapFrame.cpp
  ${rover_gui_plugin_MOCS}
  ${rover_gui_plugin_UIS_H}
  ${OpenCV_LIBS}
)

target_link_libraries(
  rqt_rover_gui
  ${catkin_LIBRARIES}
)

catkin_python_setup()
