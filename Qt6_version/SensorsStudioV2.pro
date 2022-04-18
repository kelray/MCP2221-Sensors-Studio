QT       += core gui #location

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets charts multimedia testlib positioning serialport sql

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp\
    MPU9250.cpp \
    error_list.cpp \

HEADERS += \
    mainwindow.h\
    GetErrName.h \
    MPU9250.h \
    mainwindow.h \
    mcp2221_dll_um.h \

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#INCLUDEPATH += "$$PWD\MapGraphics"  #uncomment forQt5
INCLUDEPATH += "$$PWD\MapGraphics-Qt6Modifications\MapGraphics"  #Uncomment for Qt6

#Linkage for MapGraphics shared library
win32:CONFIG(release, debug|release): LIBS += "$$PWD\\mcp2221_dll_um_x64.lib"\
                                              "$$PWD\\MapGraphics_BUILD\\LIB_BINARY\\MSVC2019_64bit-Release\\libMapGraphics.a"
                                              
else:win32:CONFIG(debug, debug|release): LIBS += "$$PWD\\mcp2221_dll_um_x64.lib"\
                                                 "$$PWD\\MapGraphics_BUILD\\LIB_BINARY\\MSVC2019_64bit-Release\\libMapGraphics.a"
