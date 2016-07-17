#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcl_visualizer
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    cloudfilter.cpp \
    segmentation.cpp \
    KinectV2Interface.cpp \
    CloudTransformation.cpp \
    miscellaneous.cpp \
    threedimentionalbpp.cpp

HEADERS  += pclviewer.h \
    cloudfilter.h \
    segmentation.h \
    KinectV2Interface.h \
    CloudTransformation.h \
    miscellaneous.h \
    threedimentionalbpp.h

FORMS    += pclviewer.ui
