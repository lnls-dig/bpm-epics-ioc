TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

PYTHON=python

LIBRARY_IOC += BPMSupport
BPMSupport_SRCS += convert.cpp
BPMSupport_SRCS += cordic.cpp
BPMSupport_SRCS += numeric.cpp
BPMSupport_SRCS += drvBPM.cpp
BPMSupport_LIBS += asyn
BPMSupport_LIBS += ADBase
BPMSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = BPM
# BPM.dbd will be created and installed
DBD += $(PROD_IOC).dbd
$(PROD_IOC)_DBD += BPMSupport.dbd
DBD += BPMSupport.dbd

# Include dbd files from all support applications:
BPM_DBD += base.dbd
BPM_DBD += asyn.dbd
BPM_DBD += FFTRecords.dbd

# BPM_registerRecordDeviceDriver.cpp derives from BPM.dbd
BPM_SRCS += BPM_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
BPM_SRCS_DEFAULT += BPMMain.cpp
BPM_SRCS_vxWorks += -nil-

# Add all the support libraries needed by this IOC
BPM_LIBS += BPMSupport
BPM_LIBS += asyn
BPM_LIBS += autosave
BPM_LIBS += FFTRecords

# Link to the EPICS Base libraries
BPM_LIBS += $(EPICS_BASE_IOC_LIBS)

# Link to system libraries
BPM_SYS_LIBS += bpmclient
BPM_SYS_LIBS += acqclient
BPM_SYS_LIBS += halcsclient
BPM_SYS_LIBS += errhand
BPM_SYS_LIBS += hutils
BPM_SYS_LIBS += mlm
BPM_SYS_LIBS += czmq
BPM_SYS_LIBS += zmq
# FIXME: Why does epics does not include these libs?
BPM_SYS_LIBS += pthread
BPM_SYS_LIBS += readline
BPM_SYS_LIBS += m
BPM_SYS_LIBS += rt
BPM_SYS_LIBS += dl
BPM_SYS_LIBS += gcc

# System header files
BPM_INCLUDES += -I/usr/include

# Compile test program
PROD += BPMCheckInit
BPMCheckInit_SRCS += BPMCheckInit.c

# Link to system libraries
BPMCheckInit_SYS_LIBS += bpmclient
BPMCheckInit_SYS_LIBS += acqclient
BPMCheckInit_SYS_LIBS += halcsclient
BPMCheckInit_SYS_LIBS += errhand
BPMCheckInit_SYS_LIBS += hutils
BPMCheckInit_SYS_LIBS += mlm
BPMCheckInit_SYS_LIBS += czmq
BPMCheckInit_SYS_LIBS += zmq
# FIXME: Why does epics does not include these libs?
BPMCheckInit_SYS_LIBS += pthread
BPMCheckInit_SYS_LIBS += readline
BPMCheckInit_SYS_LIBS += m
BPMCheckInit_SYS_LIBS += rt
BPMCheckInit_SYS_LIBS += dl
BPMCheckInit_SYS_LIBS += gcc

# CXX Compiler flags
USR_CXXFLAGS = -std=gnu++0x -DMLM_BUILD_DRAFT_API -D__BOARD_AFCV3_1__
USR_CFLAGS = -DMLM_BUILD_DRAFT_API -D__BOARD_AFCV3_1__

include $(AREA_DETECTOR_COMPAT)/ADApp/commonDriverMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# The numeric module uses a pre-built lookup table.
numeric.o: numeric-lookup.h

%.h: ../%.py
	$(PYTHON) $< > $@
