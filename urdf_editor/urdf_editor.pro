#-------------------------------------------------
#
# Project created by QtCreator 2015-08-06T12:13:25
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = industrial_robot_builder
TEMPLATE = app

include(qtpropertybrowser.pri)

SOURCES += \
    src/main.cpp \
    src/urdf_editor.cpp \
    src/joint_tree_property.cpp \
    src/common.cpp \
    src/link_tree_property.cpp

HEADERS  += \
    include/urdf_editor/urdf_editor.h \
    include/urdf_editor/joint_tree_property.h \
    include/urdf_editor/common.h \
    include/urdf_editor/link_tree_property.h

FORMS    += ui/industrial_robot_builder.ui

LIBS += -lurdfdom_model

DISTFILES += \
    config.pri
