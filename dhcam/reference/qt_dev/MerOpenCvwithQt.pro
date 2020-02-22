#-------------------------------------------------
#
# Project created by QtCreator 2018-06-27T10:36:37
#
#-------------------------------------------------

INCLUDEPATH += ${GENICAM_ROOT_V2_3}/library/CPP/include
INCLUDEPATH += ${GENICAM_ROOT_V2_3}/../../sdk/include

#LIBS += -lgxiapi -ldximageproc\
#-L${GENICAM_ROOT_V2_3}/bin/Linux32_i86 \
#-lGCBase_gcc40_v2_3 -lGenApi_gcc40_v2_3 -llog4cpp_gcc40_v2_3 -lLog_gcc40_v2_3 -lMathParser_gcc40_v2_3

 LIBS +=-lgxiapi -ldximageproc -lpthread \
                      -L$(GENICAM_ROOT_V2_3)/bin/Linux64_x64 \
                      -L$(GENICAM_ROOT_V2_3)/bin/Linux64_x64/GenApi/Generic \
                      -L/usr/local/lib \
                  -lGCBase_gcc40_v2_3 -lGenApi_gcc40_v2_3 -llog4cpp_gcc40_v2_3 -lLog_gcc40_v2_3 -lMathParser_gcc40_v2_3 \
                  -lopencv_highgui -lopencv_core -lopencv_imgproc

#opencv header

INCLUDEPATH +=usr/local/include \
usr/local/include/opencv \
usr/local/include/opencv2

#opencv lib

LIBS+=/usr/local/lib/libopencv_highgui.so \
/usr/local/lib/libopencv_core.so



QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MerOpenCvwithQt
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui


