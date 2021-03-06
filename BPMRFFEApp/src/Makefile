TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC += BPMRFFESupport
BPMRFFESupport_SRCS += drvBPMRFFE.cpp
BPMRFFESupport_LIBS += asyn
BPMRFFESupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = BPMRFFE
# BPM.dbd will be created and installed
DBD += $(PROD_IOC).dbd
$(PROD_IOC)_DBD += BPMRFFESupport.dbd
DBD += BPMRFFESupport.dbd

# Include dbd files from all support applications:
BPMRFFE_DBD += base.dbd
BPMRFFE_DBD += asyn.dbd
BPMRFFE_DBD += devIocStats.dbd

# BPMRFFE_registerRecordDeviceDriver.cpp derives from BPM.dbd
BPMRFFE_SRCS += BPMRFFE_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
BPMRFFE_SRCS_DEFAULT += BPMRFFEMain.cpp
BPMRFFE_SRCS_vxWorks += -nil-

# Add all the support libraries needed by this IOC
BPMRFFE_LIBS += BPMRFFESupport
BPMRFFE_LIBS += asyn
BPMRFFE_LIBS += autosave
BPMRFFE_LIBS += FFTRecords
BPMRFFE_LIBS += devIocStats

# Link to the EPICS Base libraries
BPMRFFE_LIBS += $(EPICS_BASE_IOC_LIBS)

# Link to system libraries
BPMRFFE_SYS_LIBS += halcsclient
BPMRFFE_SYS_LIBS += errhand
BPMRFFE_SYS_LIBS += hutils
BPMRFFE_SYS_LIBS += mlm
BPMRFFE_SYS_LIBS += czmq
BPMRFFE_SYS_LIBS += zmq
# FIXME: Why does epics does not include these libs?
BPMRFFE_SYS_LIBS += pthread
BPMRFFE_SYS_LIBS += readline
BPMRFFE_SYS_LIBS += m
BPMRFFE_SYS_LIBS += rt
BPMRFFE_SYS_LIBS += dl
BPMRFFE_SYS_LIBS += gcc

# Compile test program
PROD += BPMRFFECheckInit
BPMRFFECheckInit_SRCS += BPMRFFECheckInit.c

# Link to system libraries
BPMRFFECheckInit_SYS_LIBS += bpmclient
BPMRFFECheckInit_SYS_LIBS += acqclient
BPMRFFECheckInit_SYS_LIBS += halcsclient
BPMRFFECheckInit_SYS_LIBS += errhand
BPMRFFECheckInit_SYS_LIBS += hutils
BPMRFFECheckInit_SYS_LIBS += mlm
BPMRFFECheckInit_SYS_LIBS += czmq
BPMRFFECheckInit_SYS_LIBS += zmq
# FIXME: Why does epics does not include these libs?
BPMRFFECheckInit_SYS_LIBS += pthread
BPMRFFECheckInit_SYS_LIBS += readline
BPMRFFECheckInit_SYS_LIBS += m
BPMRFFECheckInit_SYS_LIBS += rt
BPMRFFECheckInit_SYS_LIBS += dl
BPMRFFECheckInit_SYS_LIBS += gcc

# System header files and "any" implementation.
USR_CXXFLAGS += -I/usr/include -I$(TOP)/foreign/any

# CXX Compiler flags
USR_CXXFLAGS += -std=gnu++11 -DMLM_BUILD_DRAFT_API -D__BOARD_AFCV3_1__
USR_CFLAGS += -DMLM_BUILD_DRAFT_API -D__BOARD_AFCV3_1__

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
