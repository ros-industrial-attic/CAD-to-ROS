cmake_minimum_required(VERSION 2.8.3)
project(urdf_builder)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_ros_visualization
    qt_property_browser
    roscpp
    rviz)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

set(editor_SRCS
  src/common.cpp
  src/main.cpp
  src/my_rviz.cpp
  src/urdf_editor.cpp

  # properties
  src/joint_axis_property.cpp
  src/joint_calibration_property.cpp
  src/joint_dynamics_property.cpp
  src/joint_limits_property.cpp
  src/joint_mimic_property.cpp
  src/joint_property.cpp
  src/joint_safety_property.cpp
  src/link_collision_property.cpp
  src/link_geometry_property.cpp
  src/link_inertial_property.cpp
  src/link_new_material_property.cpp
  src/link_property.cpp
  src/link_visual_property.cpp
  src/origin_property.cpp
  src/urdf_property.cpp
)

set(editor_HDRS
  include/urdf_editor/my_rviz.h
  include/urdf_editor/urdf_editor.h

  # properties
  include/urdf_editor/joint_axis_property.h
  include/urdf_editor/joint_calibration_property.h
  include/urdf_editor/joint_dynamics_property.h
  include/urdf_editor/joint_limits_property.h
  include/urdf_editor/joint_mimic_property.h
  include/urdf_editor/joint_property.h
  include/urdf_editor/joint_safety_property.h
  include/urdf_editor/link_collision_property.h
  include/urdf_editor/link_geometry_property.h
  include/urdf_editor/link_inertial_property.h
  include/urdf_editor/link_new_material_property.h
  include/urdf_editor/link_property.h
  include/urdf_editor/link_visual_property.h
  include/urdf_editor/origin_property.h
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
  CATKIN_DEPENDS
    moveit_ros_visualization
    qt_property_browser
    roscpp
    rviz
  DEPENDS
    urdfdom
)

qt4_wrap_cpp(editor_MOCS ${editor_HDRS})
qt4_wrap_ui(editor_UIS_H ${editor_UIS})

include_directories(
  ${editor_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${urdfdom_INCLUDE_DIR}
  ${urdfdom_headers_INCLUDE_DIR})

add_executable(${PROJECT_NAME}
  ${editor_UIS_H}
  ${editor_MOCS}
  ${editor_SRCS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY})

## Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
