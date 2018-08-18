cmake_minimum_required(VERSION 2.8.3)
project(rqt_state_machine)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
  std_msgs
  state_machine_msgs
  orb_slam_2_ros
#  freespace_ros
)

find_package(Qt5Widgets REQUIRED)

set(rqt_state_machine_SRCS
  src/rqt_state_machine/state_machine_controller.cpp
)

set(rqt_state_machine_HDRS
  include/rqt_state_machine/state_machine_controller.h
)

set(rqt_state_machine_UIS
  src/rqt_state_machine/state_machine_controller.ui
)

set(rqt_state_machine_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${rqt_state_machine_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp std_msgs
)

qt5_wrap_cpp(rqt_state_machine_MOCS ${rqt_state_machine_HDRS})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

qt5_wrap_ui(rqt_state_machine_UIS_H ${rqt_state_machine_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${rqt_state_machine_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${rqt_state_machine_SRCS} ${rqt_state_machine_MOCS} ${rqt_state_machine_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
