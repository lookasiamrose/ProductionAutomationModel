#-------------------------------------------------
#
# Project created by QtCreator 2016-12-31T12:35:19
#
#-------------------------------------------------

QT       += core gui
QT       += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RoboArmController
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    roboarm.cpp \
    led_indicator/ledindicator.cpp \
    uartcontroller.cpp

HEADERS  += mainwindow.h \
    roboarm.h \
    led_indicator/ledindicator.h \
    uartcontroller.h

FORMS    += mainwindow.ui
