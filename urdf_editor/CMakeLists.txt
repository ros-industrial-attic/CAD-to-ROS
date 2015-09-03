cmake_minimum_required(VERSION 2.8.3)
project(urdf_builder)
find_package(catkin REQUIRED COMPONENTS rviz roscpp qt_property_browser)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(urdfdom REQUIRED)

set(editor_SRCS
      src/main.cpp
      src/urdf_editor.cpp
      src/common.cpp
      src/joint_property.cpp
      src/link_property.cpp
      src/urdf_property.cpp
)

set(editor_HDRS
      include/urdf_editor/urdf_editor.h
      include/urdf_editor/joint_property.h
      include/urdf_editor/link_property.h
      include/urdf_editor/urdf_property.h
)

set(editor_UIS
      include/urdf_editor/industrial_robot_builder.ui
)

set(editor_INCLUDE_DIRECTORIES
      include
      ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${editor_INCLUDE_DIRECTORIES}
  LIBRARIES
  CATKIN_DEPENDS roscpp rviz qt_property_browser
  DEPENDS urdfdom
)

qt4_wrap_cpp(editor_MOCS ${editor_HDRS})
qt4_wrap_ui(editor_UIS_H ${editor_UIS})

include_directories(${editor_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${urdfdom_INCLUDE_DIR})

add_executable(urdf_builder
                  ${editor_UIS_H}
                  ${editor_MOCS}
                  ${editor_SRCS})

target_link_libraries(urdf_builder
                        ${catkin_LIBRARIES}
                        ${urdfdom_LIBRARIES}
                        ${QT_LIBRARIES}
                        ${QT_QTCORE_LIBRARY}
                        ${QT_QTGUI_LIBRARY})

## Install
#install(TARGETS urdf_builder DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
