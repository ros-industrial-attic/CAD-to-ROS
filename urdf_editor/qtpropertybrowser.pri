include(common.pri)
greaterThan(QT_MAJOR_VERSION, 4): QT *= widgets
INCLUDEPATH += $$PWD
DEPENDPATH += $$PWD

qtpropertybrowser-uselib:!qtpropertybrowser-buildlib {
    LIBS += -L$$QTPROPERTYBROWSER_LIBDIR -l$$QTPROPERTYBROWSER_LIBNAME
} else {
    DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0
    SOURCES += $$PWD/src/qtpropertybrowser/qtpropertybrowser.cpp \
            $$PWD/src/qtpropertybrowser/qtpropertymanager.cpp \
            $$PWD/src/qtpropertybrowser/qteditorfactory.cpp \
            $$PWD/src/qtpropertybrowser/qtvariantproperty.cpp \
            $$PWD/src/qtpropertybrowser/qttreepropertybrowser.cpp \
            $$PWD/src/qtpropertybrowser/qtbuttonpropertybrowser.cpp \
            $$PWD/src/qtpropertybrowser/qtgroupboxpropertybrowser.cpp \
            $$PWD/src/qtpropertybrowser/qtpropertybrowserutils.cpp
    HEADERS += $$PWD/include/urdf_editor/qtpropertybrowser/qtpropertybrowser.h \
            $$PWD/include/urdf_editor/qtpropertybrowser/qtpropertymanager.h \
            $$PWD/include/urdf_editor/qtpropertybrowser/qteditorfactory.h \
            $$PWD/include/urdf_editor/qtpropertybrowser/qtvariantproperty.h \
            $$PWD/include/urdf_editor/qtpropertybrowser/qttreepropertybrowser.h \
            $$PWD/include/urdf_editor/qtpropertybrowser/qtbuttonpropertybrowser.h \
            $$PWD/include/urdf_editor/qtpropertybrowser/qtgroupboxpropertybrowser.h
    RESOURCES += $$PWD/include/urdf_editor/qtpropertybrowser/qtpropertybrowser.qrc
}

win32 {
    contains(TEMPLATE, lib):contains(CONFIG, shared):DEFINES += QT_QTPROPERTYBROWSER_EXPORT
    else:qtpropertybrowser-uselib:DEFINES += QT_QTPROPERTYBROWSER_IMPORT
}

HEADERS += \
    $$PWD/include/urdf_editor/qtpropertybrowser/qtpropertybrowserutils_p.h
