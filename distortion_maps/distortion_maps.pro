#-------------------------------------------------
#
# Calibrator/libCalib distortion map example
#
#-------------------------------------------------

QT       -= core gui

TARGET = distortion_maps
TEMPLATE = app

CONFIG += c++14

SOURCES += \
        main.cpp

win32 {
    CONFIG += static_runtime
}


include(../dependencies.pri)
