#-------------------------------------------------
#
# Project created by QtCreator 2015-08-06T12:13:25
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = industrial_robot_builder
TEMPLATE = app

include(qtpropertybrowser/qtpropertybrowser.pri)

SOURCES += main.cpp\
        industrial_robot_builder.cpp

HEADERS  += industrial_robot_builder.h

FORMS    += industrial_robot_builder.ui

LIBS += -lurdfdom_model

DISTFILES += \
    config.pri
