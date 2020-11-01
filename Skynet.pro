#-------------------------------------------------
#
# Project created by QtCreator 
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Skynet
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        kinematics.cpp \
        main.cpp \
        cmainwindow.cpp \
        c3dwidget.cpp

HEADERS += \
        cmainwindow.h \
        c3dwidget.h \
        kinematics.h \
        makebottle.h

FORMS += \
        cmainwindow.ui

INCLUDEPATH +=  /usr/include/ \
                /home/user/Downloads/opencascade-7.4.0/inc/ \
                /usr/local/include/kdl/ \
                /usr/include/eigen3/ \
                /usr/local/lib/ \


    #/home/user/orocos_kinematics_dynamics/orocos_kdl/src/ \

LIBS += -L/usr/local/lib/ \

LIBS += -lTKernel -lTKMath -lTKService -lTKV3d -lTKOpenGl \
        -lTKBRep -lTKIGES -lTKSTL -lTKVRML -lTKSTEP -lTKSTEPAttr -lTKSTEP209 \
        -lTKSTEPBase -lTKGeomBase -lTKGeomAlgo -lTKG3d -lTKG2d \
        -lTKXSBase -lTKShHealing -lTKHLR -lTKTopAlgo -lTKMesh -lTKPrim \
        -lTKCDF -lTKBool -lTKBO -lTKFillet -lTKOffset \
        -lorocos-kdl -lTKXDESTEP \

#lcnc
INCLUDEPATH +=  /usr/lib/ \
                /usr/lib/linuxcnc/modules/ \
                /usr/include/ \
                /usr/include/linuxcnc/
LIBS += -llinuxcnchal -lethercat -Iinclude -Isrc/emc/rs274ngc -Llib -lnml -llinuxcnc -llinuxcnchal -llinuxcncini -lposemath


RESOURCES += \
    res.qrc

DISTFILES += \
    trash

# Copy the robot directory to your compile directory.

# Install linuxcnc or better install the Debian Ethercat RTOS iso from https://github.com/grotius-cnc/ ..
# This iso contain's the auto ethercat installation + the realtime hal layer.

# Install KDL kinematics dependencies   $ sudo apt-get install libeigen3-dev libcppunit-dev
# Install opencascade                   $ sudo apt-get install libocct-* or better if you have more time: install https://github.com/grotius-cnc/orocos_kinematics_dynamics.git
# Install assimp                        $ sudo apt-get install assimp* (assimp will be removed, we don't need it at the moment.)
















