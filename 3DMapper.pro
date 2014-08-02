#-------------------------------------------------
#
# Project created by QtCreator 2014-07-10T17:16:03
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3DMapper
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
        viewer.cpp \
        cloudmerger.cpp \
    cameraopenni.cpp \
    transformmatrixdialog.cpp \
    mapperconfig.cpp \
    test_cloudmerger.cpp \
    directoryreader.cpp

HEADERS  += mainwindow.h \
            viewer.h \
            cloudmerger.h \
    cameraopenni.h \
    ICamera.h \
    transformmatrixdialog.h \
    mapperconfig.h \
    directoryreader.h

FORMS    += mainwindow.ui \
    transformmatrixdialog.ui



INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/vtk-5.8
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/ni

QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp


unix:!macx: LIBS += -lboost_thread

unix:!macx: LIBS += -lboost_system

unix:!macx: LIBS += -lpcl_visualization

unix:!macx: LIBS += -lpcl_common

unix:!macx: LIBS += -lpcl_io




unix|win32: LIBS += -lvtkRendering

unix|win32: LIBS += -lvtkCommon


unix|win32: LIBS += -lvtkFiltering



unix:!macx: LIBS += -lXnCore

unix:!macx: LIBS += -lOpenNI

unix:!macx: LIBS += -lQVTK

unix:!macx: LIBS += -lpcl_registration

unix:!macx: LIBS += -lpcl_search

unix:!macx: LIBS += -lpcl_filters

unix:!macx: LIBS += -lpcl_features

unix:!macx: LIBS += -lpcl_segmentation

unix:!macx: LIBS += /usr/local/lib/libyaml-cpp.a

OTHER_FILES += \
    mapper_config.yaml

install_it.path = $$OUT_PWD
install_it.files = mapper_config.yaml

INSTALLS += \
    install_it


