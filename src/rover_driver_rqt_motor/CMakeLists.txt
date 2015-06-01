cmake_minimum_required(VERSION 2.8.3)
project(rover_driver_rqt_motor)

find_package(catkin REQUIRED COMPONENTS 
  rqt_gui
  rqt_gui_cpp
)

find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
)

catkin_package(
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp std_msgs geometry_msgs
)

qt4_wrap_cpp(
  rover_driver_rqt_motor_MOCS
  include/rover_motor.h
)

qt4_wrap_ui(
  rover_driver_rqt_motor_UIS_H
  src/rover_motor.ui
)

include(
  ${QT_USE_FILE}
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(
  rover_driver_rqt_motor
  src/rover_motor.cpp
  ${rover_driver_rqt_motor_MOCS}
  ${rover_driver_rqt_motor_UIS_H}
)

target_link_libraries(
  rover_driver_rqt_motor
  ${catkin_LIBRARIES}
)

add_executable(
  joystick_driver
  src/joystick_driver.cpp
)

target_link_libraries(
  joystick_driver
  ${catkin_LIBRARIES}
)
