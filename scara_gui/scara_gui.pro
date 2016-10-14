TEMPLATE = app
TARGET = scara_gui

QT = core gui widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
SOURCES += \
    main.cpp \
    window.cpp

HEADERS += \
    window.h
