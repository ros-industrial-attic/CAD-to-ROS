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

# TODO: hack for catkin_tools / travis: do not place mocs in build dir
set(qtpe_MOC_DIR ${CATKIN_DEVEL_PREFIX}/${PROJECT_NAME}/moc)

set(qtpropertyeditor_INCLUDE_DIRECTORIES
      src
      ${qtpe_MOC_DIR}
      ${QT_INCLUDE_DIR}
)

qt4_generate_moc(
  src/qtpropertybrowser.h
  ${qtpe_MOC_DIR}/moc_qtpropertybrowser.cpp
)
qt4_generate_moc(
  src/qtpropertymanager.h
  ${qtpe_MOC_DIR}/moc_qtpropertymanager.cpp
)
qt4_generate_moc(
  src/qteditorfactory.h
  ${qtpe_MOC_DIR}/moc_qteditorfactory.cpp
)
qt4_generate_moc(
  src/qtvariantproperty.h
  ${qtpe_MOC_DIR}/moc_qtvariantproperty.cpp
)
qt4_generate_moc(
  src/qttreepropertybrowser.h
  ${qtpe_MOC_DIR}/moc_qttreepropertybrowser.cpp
)
qt4_generate_moc(
  src/qtbuttonpropertybrowser.h
  ${qtpe_MOC_DIR}/moc_qtbuttonpropertybrowser.cpp
)
qt4_generate_moc(
  src/qtgroupboxpropertybrowser.h
  ${qtpe_MOC_DIR}/moc_qtgroupboxpropertybrowser.cpp
)
qt4_generate_moc(
  src/qtpropertymanager.cpp
  ${qtpe_MOC_DIR}/qtpropertymanager.moc
)
qt4_generate_moc(
  src/qteditorfactory.cpp
  ${qtpe_MOC_DIR}/qteditorfactory.moc
)
qt4_generate_moc(
  src/qttreepropertybrowser.cpp
  ${qtpe_MOC_DIR}/qttreepropertybrowser.moc
)

set(
  qtpropertyeditor_HEADERS_ONLY_MOC
  ${qtpe_MOC_DIR}/moc_qtpropertybrowser.cpp
  ${qtpe_MOC_DIR}/moc_qtpropertymanager.cpp
  ${qtpe_MOC_DIR}/moc_qteditorfactory.cpp
  ${qtpe_MOC_DIR}/moc_qtvariantproperty.cpp
  ${qtpe_MOC_DIR}/moc_qttreepropertybrowser.cpp
  ${qtpe_MOC_DIR}/moc_qtbuttonpropertybrowser.cpp
  ${qtpe_MOC_DIR}/moc_qtgroupboxpropertybrowser.cpp
  ${qtpe_MOC_DIR}/qtpropertymanager.moc
  ${qtpe_MOC_DIR}/qteditorfactory.moc
  ${qtpe_MOC_DIR}/qttreepropertybrowser.moc
)

set_source_files_properties(
  ${qtpropertyeditor_HEADERS_ONLY_MOC}
  PROPERTIES
    HEADER_FILE_ONLY true
)

qt4_generate_moc(
  src/qtpropertybrowserutils_p.h
  ${qtpe_MOC_DIR}/moc_qtpropertybrowserutils_p.cpp
)

set(
  qtpropertyeditor_HEADERS_MOC
  ${qtpe_MOC_DIR}/moc_qtpropertybrowserutils_p.cpp
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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  DEPENDS Qt4
  CFG_EXTRAS qt_property_browser-extras.cmake
)


include_directories(${qtpropertyeditor_INCLUDE_DIRECTORIES})

add_library(${PROJECT_NAME}
                  ${qtpropertyeditor_MOC}
                  ${qtpropertyeditor_SRCS}
                  ${qtpropertyeditor_RESOURCES})

target_link_libraries(${PROJECT_NAME}
                        ${QT_LIBRARIES}
                        ${QT_QTCORE_LIBRARY}
                        ${QT_QTGUI_LIBRARY})

ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_DEFINITIONS(-DQT_DLL)
#ADD_DEFINITIONS(-DQT_QTPROPERTYBROWSER_EXPORT)
#ADD_DEFINITIONS(-DQT_PLUGIN)
#ADD_DEFINITIONS(-DQT_SHARED)

## Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# TODO: remove hack for catkin_tools / travis
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")
