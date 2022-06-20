#-------------------------------------------------
#
# Calibrator/libCalib unprojection maps example
#
#-------------------------------------------------

QT       -= core gui

TARGET = unprojection_maps
TEMPLATE = app

CONFIG += c++14

SOURCES += main.cpp

win32 {
    CONFIG += static_runtime
}


include(../dependencies.pri)
