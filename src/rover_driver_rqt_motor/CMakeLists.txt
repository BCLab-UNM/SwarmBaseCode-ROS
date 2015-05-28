cmake_minimum_required(VERSION 2.8.3)

project(rover_driver_rqt_motor)

find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rover_driver_rqt_motor_SRCS
  src/rover_driver_rqt_motor/rover_motor.cpp
)

set(rover_driver_rqt_motor_HDRS
  include/rover_driver_rqt_motor/rover_motor.h
)

set(rover_driver_rqt_motor_UIS
  src/rover_driver_rqt_motor/rover_motor.ui
)

set(rover_driver_rqt_motor_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rover_driver_rqt_motor_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp std_msgs geometry_msgs
)
catkin_python_setup()

qt4_wrap_cpp(rover_driver_rqt_motor_MOCS ${rover_driver_rqt_motor_HDRS})
qt4_wrap_ui(rover_driver_rqt_motor_UIS_H ${rover_driver_rqt_motor_UIS})

include_directories(${rover_driver_rqt_motor_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rover_driver_rqt_motor_SRCS} ${rover_driver_rqt_motor_MOCS} ${rover_driver_rqt_motor_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rover_driver_rqt_motor
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rover_driver_rqt_motor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(joystick_driver src/joystick_driver.cpp)
target_link_libraries(joystick_driver ${catkin_LIBRARIES})  



