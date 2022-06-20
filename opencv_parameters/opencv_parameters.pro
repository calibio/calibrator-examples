#-------------------------------------------------
#
# libCalib example OpenCV parameter import
#
#-------------------------------------------------

QT       -= core gui

TARGET = opencv_parameters
TEMPLATE = app

CONFIG += c++14

SOURCES += \
        main.cpp

win32 {
    CONFIG += static_runtime
}


include(../dependencies.pri)
