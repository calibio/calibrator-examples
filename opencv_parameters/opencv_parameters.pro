#-----------------------------------------------------
#
# Calibrator/libCalib example OpenCV parameter import
#
#-----------------------------------------------------

QT       -= core gui

TARGET = opencv_parameters
TEMPLATE = app

CONFIG += c++14

HEADERS += json.hpp \
    readCalibParameters.h

SOURCES += \
        main.cpp \
        readCalibParameters.cpp

win32 {
    CONFIG += static_runtime
}

DEFINES += "SRCDIR=\\\"$$PWD\\\""

# OpenCV
unix:!macos {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv4
}
win32{
    INCLUDEPATH += C:\opencv\build\include
    LIBS += -LC:\opencv\build\x64\vc15\lib -lopencv_world410
}
