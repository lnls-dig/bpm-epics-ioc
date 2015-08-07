TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
# Foreign folder location
FOREIGN=foreign
PYTHON=python

LIBRARY_IOC += BPMSupport
#BPMSupport_SRCS += foreign/convert.cpp
#BPMSupport_SRCS += foreign/cordic.cpp
#BPMSupport_SRCS += foreign/numeric.cpp
BPMSupport_SRCS += convert.cpp
BPMSupport_SRCS += cordic.cpp
BPMSupport_SRCS += numeric.cpp
BPMSupport_SRCS += drvBPM.cpp
BPMSupport_LIBS += asyn
BPMSupport_LIBS += ADBase
#BPMSupport_LIBS += bpmclient
#BPMSupport_LIBS += errhand
#BPMSupport_LIBS += hutils
#BPMSupport_LIBS += mlm
#BPMSupport_LIBS += czmq
#BPMSupport_LIBS += zmq
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
BPM_LIBS += mca
BPM_LIBS += pv
BPM_LIBS += FFTRecords

# Link to the EPICS Base libraries
BPM_LIBS += $(EPICS_BASE_IOC_LIBS)

# Link to system libraries
BPM_SYS_LIBS += bpmclient
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
#BPM_INCLUDES += -Iforeign

# CXX Compiler flags
USR_CXXFLAGS = -std=gnu++0x

include $(AREA_DETECTOR)/ADApp/commonDriverMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# The numeric module uses a pre-built lookup table.
#$(FOREIGN)/numeric.o: numeric-lookup.h
numeric.o: numeric-lookup.h

#%.h: ../$(FOREIGN)/%.py
#	$(PYTHON) $< > $@
%.h: ../%.py
	$(PYTHON) $< > $@

