QT       += core

# DEFINES += IMAGES16BIT
LIBS += -ltiff

TARGET = test_blobfinder
TEMPLATE = app

LIBS        += -lssl
LIBS        += -lcrypto
LIBS        += -L/usr/local/astrometryGit/lib -lastrometry

INCLUDEPATH = ../astrometry

SOURCES += main.cpp \
    ../bitsc/paramHandler.cpp \
    ../bitsc/communicator.cpp \
    ../common/cmdparser.c \
    ../common/bitserver.c \
    ../common/fifo.c \
    ../bitsc/astrometry.cpp\
    blobfinder.cpp

HEADERS += \
    blobfinder.h

