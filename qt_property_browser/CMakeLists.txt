cmake_minimum_required(VERSION 2.8.3)
project(qt_property_browser)

find_package(catkin REQUIRED)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(qtpropertyeditor_SRCS
      src/qtpropertybrowser.cpp
      src/qtpropertymanager.cpp
      src/qteditorfactory.cpp
      src/qtvariantproperty.cpp
      src/qttreepropertybrowser.cpp
      src/qtbuttonpropertybrowser.cpp
      src/qtgroupboxpropertybrowser.cpp
      src/qtpropertybrowserutils.cpp
)

set(qtpropertyeditor_INCLUDE_DIRECTORIES
      src
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/src
      ${QT_INCLUDE_DIR}
)

qt4_generate_moc(
  src/qtpropertybrowser.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowser.cpp
)
qt4_generate_moc(
  src/qtpropertymanager.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertymanager.cpp
)
qt4_generate_moc(
  src/qteditorfactory.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qteditorfactory.cpp
)
qt4_generate_moc(
  src/qtvariantproperty.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtvariantproperty.cpp
)
qt4_generate_moc(
  src/qttreepropertybrowser.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qttreepropertybrowser.cpp
)
qt4_generate_moc(
  src/qtbuttonpropertybrowser.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtbuttonpropertybrowser.cpp
)
qt4_generate_moc(
  src/qtgroupboxpropertybrowser.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtgroupboxpropertybrowser.cpp
)
qt4_generate_moc(
  src/qtpropertymanager.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertymanager.moc
)
qt4_generate_moc(
  src/qteditorfactory.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/qteditorfactory.moc
)
qt4_generate_moc(
  src/qttreepropertybrowser.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/qttreepropertybrowser.moc
)

set(
  qtpropertyeditor_HEADERS_ONLY_MOC
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowser.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertymanager.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qteditorfactory.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtvariantproperty.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qttreepropertybrowser.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtbuttonpropertybrowser.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtgroupboxpropertybrowser.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertymanager.moc
  ${CMAKE_CURRENT_BINARY_DIR}/qteditorfactory.moc
  ${CMAKE_CURRENT_BINARY_DIR}/qttreepropertybrowser.moc
)

set_source_files_properties(
  ${qtpropertyeditor_HEADERS_ONLY_MOC}
  PROPERTIES
    HEADER_FILE_ONLY true
)

qt4_generate_moc(
  src/qtpropertybrowserutils_p.h
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowserutils_p.cpp
)

set(
  qtpropertyeditor_HEADERS_MOC
  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowserutils_p.cpp
)

set(
  qtpropertyeditor_MOC
  ${qtpropertyeditor_HEADERS_MOC}
  ${qtpropertyeditor_HEADERS_ONLY_MOC}
)

qt4_add_resources(
  qtpropertyeditor_RESOURCES
  src/qtpropertybrowser.qrc
)

catkin_package(
  INCLUDE_DIRS ${qtpropertyeditor_INCLUDE_DIRECTORIES}
  LIBRARIES qt_property_browser
  CATKIN_DEPENDS 
  DEPENDS Qt4
)


include_directories(${qtpropertyeditor_INCLUDE_DIRECTORIES})

add_library(qt_property_browser
                  ${qtpropertyeditor_MOC}
                  ${qtpropertyeditor_SRCS}
                  ${qtpropertyeditor_RESOURCES})

target_link_libraries(qt_property_browser
                        ${QT_LIBRARIES}
                        ${QT_QTCORE_LIBRARY}
                        ${QT_QTGUI_LIBRARY})

ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_DEFINITIONS(-DQT_DLL)
#ADD_DEFINITIONS(-DQT_QTPROPERTYBROWSER_EXPORT)
#ADD_DEFINITIONS(-DQT_PLUGIN)
#ADD_DEFINITIONS(-DQT_SHARED)

## Install
install(TARGETS qt_property_browser DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
